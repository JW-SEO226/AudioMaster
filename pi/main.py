import numpy as np
import tensorflow_hub as hub
import sounddevice as sd
import serial
import time
from collections import deque, Counter

# =========================================================
# ✅ 라즈베리파이 UART 설정 (물리적 8, 10번 핀 사용)
# 라즈베리파이5의 경우 GPIO 14/15는 /dev/ttyAMA0 에 매핑됩니다.
# =========================================================
ser = serial.Serial(
    "/dev/ttyAMA0",   # Pi5 GPIO UART 포트 유지
    115200,
    timeout=0.1
)

# =========================================================
# YAMNet 로드
# =========================================================
print("YAMNet 모델 로딩 중...")
yamnet = hub.load("https://tfhub.dev/google/yamnet/1")
print("✅ YAMNet 로드 완료!")

# =========================================================
# Audio / Window 설정
# =========================================================
SAMPLE_RATE = 16000
WINDOW_SEC  = 0.96
HOP_SEC     = 0.24
WINDOW_SIZE = int(SAMPLE_RATE * WINDOW_SEC)
HOP_SIZE    = int(SAMPLE_RATE * HOP_SEC)

CONF_HISTORY  = 10
STATE_HISTORY = 10
TREND_HISTORY = 10

# 휴면 모드
WAKEUP_RMS_DB  = -30.0
KEEP_AWAKE_SEC = 3.0
last_loud_time = 0.0
is_awake       = False

# YAMNet 클래스
SIREN_CLASSES = [316, 317, 318, 319, 390]
HORN_CLASSES  = [302]

TH_SIREN_ACTIVE = 0.35
TH_HORN_ACTIVE  = 0.25

S_SIREN_ENTER_NEAR     = 0.60
S_SIREN_EXIT_NEAR      = 0.45
S_SIREN_ENTER_APPROACH = 0.35
S_SIREN_EXIT_APPROACH  = 0.25
S_HORN_ENTER           = 0.25
S_HORN_EXIT            = 0.15

R_NEAR_ENTER = -20.0
R_NEAR_EXIT  = -27.0
D_APPROACH_ENTER = 3.0
D_APPROACH_EXIT  = 1.0

C_SIREN_NEAR_ENTER     = 0.60
C_SIREN_NEAR_EXIT      = 0.40
C_SIREN_APPROACH_ENTER = 0.30
C_SIREN_APPROACH_EXIT  = 0.20
C_HORN_REPEAT_ENTER    = 0.30
C_HORN_REPEAT_EXIT     = 0.20

TREND_APPROACH_ENTER = 4.0
TREND_APPROACH_EXIT  = 1.5

# 버퍼
audio_buffer        = deque(maxlen=WINDOW_SIZE * 3)
siren_active_history= deque(maxlen=CONF_HISTORY)
horn_active_history = deque(maxlen=CONF_HISTORY)
siren_score_history = deque(maxlen=CONF_HISTORY)
horn_score_history  = deque(maxlen=CONF_HISTORY)
rms_db_history      = deque(maxlen=TREND_HISTORY)
drms_db_history     = deque(maxlen=TREND_HISTORY)
state_history       = deque(maxlen=STATE_HISTORY)

latest_rms_db   = -80.0
latest_drms_db  = 0.0
latest_peak     = 0

current_state       = "NORMAL"
last_sent_level     = 0
last_command_time   = 0.0
COMMAND_COOLDOWN_SEC= 0.5

# =========================================================
# Audio 콜백
# =========================================================
def audio_callback(indata, frames, time_info, status):
    mono = indata[:, 0].astype(np.float32)
    audio_buffer.extend(mono)

# =========================================================
# ✅ UART 수신 (ESP32 UART0 데이터만 파싱)
# =========================================================
def read_uart():
    global latest_rms_db, latest_drms_db, latest_peak
    try:
        while ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            if "RMS_DB=" in line and "DRMS_DB=" in line:
                parts = line.split(",")
                for p in parts:
                    p = p.strip()
                    if   p.startswith("RMS_DB="):  latest_rms_db  = float(p.split("=")[1])
                    elif p.startswith("DRMS_DB="): latest_drms_db = float(p.split("=")[1])
                    elif p.startswith("PEAK="):    latest_peak    = int(p.split("=")[1])
            # ACK 메시지 등 다른 라인은 조용히 무시
    except Exception as e:
        print(f"UART parse error: {e}")

# =========================================================
# 유틸리티 및 상태 판단 로직
# =========================================================
def get_group_score(scores_np, class_indices):
    return float(max(scores_np[:, idx].max() for idx in class_indices))

def calc_confidence(active_history):
    if not active_history: return 0.0
    return sum(active_history) / len(active_history)

def calc_rms_slope_db_per_window():
    if len(rms_db_history) < 4: return 0.0
    y = np.array(rms_db_history, dtype=np.float32)
    x = np.arange(len(y), dtype=np.float32)
    slope, _ = np.polyfit(x, y, 1)
    return float(slope)

def calc_rms_total_rise_db():
    if len(rms_db_history) < 2: return 0.0
    return float(rms_db_history[-1] - rms_db_history[0])

def majority_state():
    if not state_history: return "NORMAL"
    return Counter(state_history).most_common(1)[0][0]

def detect_semantic_transition(S_siren, S_horn, C_siren, C_horn):
    prev = list(state_history)[-5:]
    had_siren = any(s in ["FAR_SIREN","APPROACHING_SIREN","NEAR_SIREN"] for s in prev)
    had_horn  = any(s in ["SHORT_HORN","REPEATED_HORN"] for s in prev)
    siren_now = S_siren >= S_SIREN_ENTER_APPROACH or C_siren >= C_SIREN_APPROACH_ENTER
    horn_now  = S_horn  >= S_HORN_ENTER           or C_horn  >= C_HORN_REPEAT_ENTER
    if siren_now and horn_now: return "SIREN_AND_HORN"
    if had_siren and horn_now: return "SIREN_TO_HORN"
    if had_horn  and siren_now:return "HORN_TO_SIREN"
    return None

def update_state_with_hysteresis(S_siren, S_horn, R_db, D_db, C_siren, C_horn, rms_slope, rms_rise):
    global current_state
    transition = detect_semantic_transition(S_siren, S_horn, C_siren, C_horn)
    if transition in ["SIREN_AND_HORN","SIREN_TO_HORN"]:
        current_state = "COMPLEX_EMERGENCY"
        return "COMPLEX_EMERGENCY", 4, "MUTE"

    # 💡 [정우 님 수정본 적용] 히스테리시스 유지 논리 정상화
    # Exit 조건이 '모두' 충족되어야만 계속 유지(AND), 아니면 강등
    still_near = (S_siren >= S_SIREN_EXIT_NEAR and R_db >= R_NEAR_EXIT and C_siren >= C_SIREN_NEAR_EXIT)
    if current_state == "NEAR_SIREN":
        if still_near:
            return "NEAR_SIREN", 4, "MUTE"
        else:
            current_state = "NORMAL"
            return "NORMAL", 0, "NORMAL"

    if S_siren >= S_SIREN_ENTER_NEAR and R_db >= R_NEAR_ENTER and C_siren >= C_SIREN_NEAR_ENTER:
        current_state = "NEAR_SIREN"; return "NEAR_SIREN", 4, "MUTE"
    
    if current_state == "APPROACHING_SIREN" and S_siren >= S_SIREN_EXIT_APPROACH and (D_db >= D_APPROACH_EXIT or rms_rise >= TREND_APPROACH_EXIT or rms_slope > 0.0) and C_siren >= C_SIREN_APPROACH_EXIT:
        return "APPROACHING_SIREN", 3, "-10dB"
    
    if S_siren >= S_SIREN_ENTER_APPROACH and C_siren >= C_SIREN_APPROACH_ENTER and (D_db >= D_APPROACH_ENTER or rms_rise >= TREND_APPROACH_ENTER or rms_slope >= 0.5):
        current_state = "APPROACHING_SIREN"; return "APPROACHING_SIREN", 3, "-10dB"
    
    if current_state == "REPEATED_HORN" and S_horn >= S_HORN_EXIT and C_horn >= C_HORN_REPEAT_EXIT:
        return "REPEATED_HORN", 3, "-10dB"
    
    if S_horn >= S_HORN_ENTER and C_horn >= C_HORN_REPEAT_ENTER:
        current_state = "REPEATED_HORN"; return "REPEATED_HORN", 3, "-10dB"
    
    if S_horn >= S_HORN_ENTER and C_horn < C_HORN_REPEAT_ENTER:
        current_state = "SHORT_HORN"; return "SHORT_HORN", 2, "-6dB"
    
    if S_siren >= S_SIREN_ENTER_APPROACH and C_siren >= C_SIREN_APPROACH_ENTER:
        current_state = "FAR_SIREN"; return "FAR_SIREN", 1, "-3dB"
    
    if S_siren < 0.15 and S_horn < 0.10 and R_db >= -25.0:
        current_state = "LOUD_NOISE"; return "LOUD_NOISE", 1, "MONITOR"
    
    current_state = "NORMAL"; return "NORMAL", 0, "NORMAL"

def send_command(level):
    global last_sent_level
    
    # 💡 핵심: 현재 레벨이 방금 보낸 레벨과 똑같다면 굳이 또 보내지 않고 무시!
    if level == last_sent_level:
        return
        
    cmds = {4: b"DUCKING:4\n", 3: b"DUCKING:3\n", 2: b"DUCKING:2\n", 1: b"DUCKING:1\n"}
    ser.write(cmds.get(level, b"NORMAL\n"))
    last_sent_level = level

# =========================================================
# Main loop
# =========================================================
stream = sd.InputStream(
    samplerate=SAMPLE_RATE, channels=1, dtype="float32",
    blocksize=HOP_SIZE, callback=audio_callback
)
stream.start()
print("Pi 상황 판단 시스템 시작 (UART: /dev/ttyAMA0)")

last_process_time = time.time()
sleep_print_time  = 0.0

try:
    while True:
        read_uart()
        now = time.time()

        if now - last_process_time >= HOP_SEC:
            last_process_time = now

            if len(audio_buffer) < WINDOW_SIZE:
                time.sleep(0.01)
                continue

            if latest_rms_db > WAKEUP_RMS_DB:
                last_loud_time = now
            is_awake = (now - last_loud_time) < KEEP_AWAKE_SEC

            if is_awake:
                audio_window = np.array(list(audio_buffer)[-WINDOW_SIZE:], dtype=np.float32)
                scores, _, _ = yamnet(audio_window)
                scores_np = scores.numpy()
                S_siren = get_group_score(scores_np, SIREN_CLASSES)
                S_horn  = get_group_score(scores_np, HORN_CLASSES)
            else:
                S_siren = 0.0
                S_horn  = 0.0

            siren_active_history.append(1 if S_siren >= TH_SIREN_ACTIVE else 0)
            horn_active_history.append( 1 if S_horn  >= TH_HORN_ACTIVE  else 0)
            siren_score_history.append(S_siren)
            horn_score_history.append(S_horn)
            rms_db_history.append(latest_rms_db)
            drms_db_history.append(latest_drms_db)

            C_siren   = calc_confidence(siren_active_history)
            C_horn    = calc_confidence(horn_active_history)
            rms_slope = calc_rms_slope_db_per_window()
            rms_rise  = calc_rms_total_rise_db()

            state, level, action = update_state_with_hysteresis(
                S_siren, S_horn, latest_rms_db, latest_drms_db,
                C_siren, C_horn, rms_slope, rms_rise
            )
            state_history.append(state)

            if is_awake:
                print(f"\n=== [YAMNet 가동] 상태: {state} | 레벨: {level} ===")
                print(f"R_db={latest_rms_db:.2f}, S_siren={S_siren:.3f}, S_horn={S_horn:.3f}")
            else:
                if now - sleep_print_time > 3.0:
                    print(f"💤 [휴면] 대기 중... ({latest_rms_db:.2f} dB)")
                    sleep_print_time = now

            send_command(level)

        time.sleep(0.005)

except KeyboardInterrupt:
    print("\n시스템 종료")
    stream.stop()
    stream.close()
    ser.close()
