import numpy as np
import tensorflow_hub as hub
import sounddevice as sd
import serial
import time
from collections import deque, Counter

# =========================================================
# UART 설정 (수정됨)
# =========================================================
ser = serial.Serial(
    "/dev/serial0",   # 💡 ttyUSB0 가 아니라 serial0 (또는 ttyS0) 로 변경!
    115200,
    timeout=0.1
)

# =========================================================
# YAMNet 로드 (초기 1회만 실행됨)
# =========================================================
print("YAMNet 모델 로딩 중... (시간이 조금 걸립니다)")
yamnet = hub.load("https://tfhub.dev/google/yamnet/1")
print("✅ YAMNet 로드 완료!")

# =========================================================
# Audio / Window 설정
# =========================================================
SAMPLE_RATE = 16000

WINDOW_SEC = 0.96
HOP_SEC = 0.24

WINDOW_SIZE = int(SAMPLE_RATE * WINDOW_SEC)
HOP_SIZE = int(SAMPLE_RATE * HOP_SEC)

CONF_HISTORY = 10        
STATE_HISTORY = 10       
TREND_HISTORY = 10       

# =========================================================
# 💡 [핵심 추가] 소프트웨어 휴면 모드(Sleep Mode) 설정
# =========================================================
WAKEUP_RMS_DB = -30.0    # 이 수치보다 소리가 커지면 YAMNet 기상 (환경에 맞춰 조절)
KEEP_AWAKE_SEC = 3.0     # 한 번 깨어나면 소리가 작아져도 유지할 시간 (최소 3초는 감시 유지)

last_loud_time = 0.0     # 마지막으로 큰 소리가 났던 시간
is_awake = False         # 현재 YAMNet 가동 상태 플래그
# =========================================================

# =========================================================
# YAMNet class index
# =========================================================
SIREN_CLASSES = [316, 317, 318, 319, 390]
HORN_CLASSES = [302]

TH_SIREN_ACTIVE = 0.35
TH_HORN_ACTIVE = 0.25

S_SIREN_ENTER_NEAR = 0.60
S_SIREN_EXIT_NEAR = 0.45
S_SIREN_ENTER_APPROACH = 0.35
S_SIREN_EXIT_APPROACH = 0.25
S_HORN_ENTER = 0.25
S_HORN_EXIT = 0.15

R_NEAR_ENTER = -20.0     
R_NEAR_EXIT = -27.0
D_APPROACH_ENTER = 3.0
D_APPROACH_EXIT = 1.0

C_SIREN_NEAR_ENTER = 0.60
C_SIREN_NEAR_EXIT = 0.40
C_SIREN_APPROACH_ENTER = 0.30
C_SIREN_APPROACH_EXIT = 0.20
C_HORN_REPEAT_ENTER = 0.30
C_HORN_REPEAT_EXIT = 0.20

TREND_APPROACH_ENTER = 4.0    
TREND_APPROACH_EXIT = 1.5

# =========================================================
# Buffer / History
# =========================================================
audio_buffer = deque(maxlen=WINDOW_SIZE * 3)
siren_active_history = deque(maxlen=CONF_HISTORY)
horn_active_history = deque(maxlen=CONF_HISTORY)
siren_score_history = deque(maxlen=CONF_HISTORY)
horn_score_history = deque(maxlen=CONF_HISTORY)
rms_db_history = deque(maxlen=TREND_HISTORY)
drms_db_history = deque(maxlen=TREND_HISTORY)
state_history = deque(maxlen=STATE_HISTORY)

latest_rms_db = -80.0
latest_drms_db = 0.0
latest_peak = 0

current_state = "NORMAL"
last_sent_level = 0
last_command_time = 0.0
COMMAND_COOLDOWN_SEC = 0.5

# =========================================================
# Audio callback & UART 수신
# =========================================================
def audio_callback(indata, frames, time_info, status):
    mono = indata[:, 0].astype(np.float32)
    audio_buffer.extend(mono)

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
                    if p.startswith("RMS_DB="): latest_rms_db = float(p.split("=")[1])
                    elif p.startswith("DRMS_DB="): latest_drms_db = float(p.split("=")[1])
                    elif p.startswith("PEAK="): latest_peak = int(p.split("=")[1])
    except Exception as e:
        print("UART parse error:", e)

# =========================================================
# 유틸리티 함수들
# =========================================================
def get_group_score(scores_np, class_indices):
    vals = [scores_np[:, idx].max() for idx in class_indices]
    return float(max(vals))

def calc_confidence(active_history):
    if len(active_history) == 0: return 0.0
    return sum(active_history) / len(active_history)

def calc_rms_slope_db_per_window():
    if len(rms_db_history) < 4: return 0.0
    y = np.array(rms_db_history, dtype=np.float32)
    x = np.arange(len(y), dtype=np.float32)
    slope, intercept = np.polyfit(x, y, 1)
    return float(slope)

def calc_rms_total_rise_db():
    if len(rms_db_history) < 2: return 0.0
    return float(rms_db_history[-1] - rms_db_history[0])

def majority_state():
    if len(state_history) == 0: return "NORMAL"
    return Counter(state_history).most_common(1)[0][0]

def detect_semantic_transition(S_siren, S_horn, C_siren, C_horn):
    prev_states = list(state_history)[-5:]
    had_siren = any(s in ["FAR_SIREN", "APPROACHING_SIREN", "NEAR_SIREN"] for s in prev_states)
    had_horn = any(s in ["SHORT_HORN", "REPEATED_HORN"] for s in prev_states)
    siren_now = (S_siren >= S_SIREN_ENTER_APPROACH or C_siren >= C_SIREN_APPROACH_ENTER)
    horn_now = (S_horn >= S_HORN_ENTER or C_horn >= C_HORN_REPEAT_ENTER)
    
    if siren_now and horn_now: return "SIREN_AND_HORN"
    if had_siren and horn_now: return "SIREN_TO_HORN"
    if had_horn and siren_now: return "HORN_TO_SIREN"
    return None

def update_state_with_hysteresis(S_siren, S_horn, R_db, D_db, C_siren, C_horn, rms_slope, rms_rise):
    global current_state
    transition = detect_semantic_transition(S_siren, S_horn, C_siren, C_horn)

    if transition in ["SIREN_AND_HORN", "SIREN_TO_HORN"]:
        current_state = "COMPLEX_EMERGENCY"
        return "COMPLEX_EMERGENCY", 4, "MUTE 또는 -15dB"
    if current_state == "NEAR_SIREN" and (S_siren >= S_SIREN_EXIT_NEAR or R_db >= R_NEAR_EXIT or C_siren >= C_SIREN_NEAR_EXIT):
        return "NEAR_SIREN", 4, "MUTE"
    if S_siren >= S_SIREN_ENTER_NEAR and R_db >= R_NEAR_ENTER and C_siren >= C_SIREN_NEAR_ENTER:
        current_state = "NEAR_SIREN"
        return "NEAR_SIREN", 4, "MUTE"
    if current_state == "APPROACHING_SIREN" and S_siren >= S_SIREN_EXIT_APPROACH and (D_db >= D_APPROACH_EXIT or rms_rise >= TREND_APPROACH_EXIT or rms_slope > 0.0) and C_siren >= C_SIREN_APPROACH_EXIT:
        return "APPROACHING_SIREN", 3, "-10dB"
    if S_siren >= S_SIREN_ENTER_APPROACH and C_siren >= C_SIREN_APPROACH_ENTER and (D_db >= D_APPROACH_ENTER or rms_rise >= TREND_APPROACH_ENTER or rms_slope >= 0.5):
        current_state = "APPROACHING_SIREN"
        return "APPROACHING_SIREN", 3, "-10dB"
    if current_state == "REPEATED_HORN" and (S_horn >= S_HORN_EXIT and C_horn >= C_HORN_REPEAT_EXIT):
        return "REPEATED_HORN", 3, "-10dB"
    if S_horn >= S_HORN_ENTER and C_horn >= C_HORN_REPEAT_ENTER:
        current_state = "REPEATED_HORN"
        return "REPEATED_HORN", 3, "-10dB"
    if S_horn >= S_HORN_ENTER and C_horn < C_HORN_REPEAT_ENTER:
        current_state = "SHORT_HORN"
        return "SHORT_HORN", 2, "-6dB 1~2초"
    if S_siren >= S_SIREN_ENTER_APPROACH and C_siren >= C_SIREN_APPROACH_ENTER:
        current_state = "FAR_SIREN"
        return "FAR_SIREN", 1, "-3dB"
    if S_siren < 0.15 and S_horn < 0.10 and R_db >= -25.0:
        current_state = "LOUD_NOISE"
        return "LOUD_NOISE", 1, "MONITOR"
    current_state = "NORMAL"
    return "NORMAL", 0, "NORMAL"

def send_command(level):
    global last_sent_level, last_command_time
    now = time.time()
    if level == last_sent_level and (now - last_command_time) < COMMAND_COOLDOWN_SEC: return
    
    if level >= 4: ser.write(b"DUCKING:4\n")
    elif level == 3: ser.write(b"DUCKING:3\n")
    elif level == 2: ser.write(b"DUCKING:2\n")
    elif level == 1: ser.write(b"DUCKING:1\n")
    else: ser.write(b"NORMAL\n")
    
    last_sent_level = level
    last_command_time = now

# =========================================================
# Main loop
# =========================================================
stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype="float32", blocksize=HOP_SIZE, callback=audio_callback)
stream.start()

print("Pi 상황 판단 시스템 시작")

last_process_time = time.time()
sleep_print_time = 0.0

try:
    while True:
        read_uart()
        now = time.time()

        if now - last_process_time >= HOP_SEC:
            last_process_time = now

            if len(audio_buffer) < WINDOW_SIZE:
                time.sleep(0.01)
                continue

            # 💡 [핵심 연산] 1. ESP32에서 온 소리 크기를 확인하여 기상 여부 결정
            if latest_rms_db > WAKEUP_RMS_DB:
                last_loud_time = now  # 큰 소리가 나면 타이머 리셋
            
            # 마지막으로 큰 소리가 난 지 KEEP_AWAKE_SEC(3초) 이내라면 깨어있는 상태 유지
            is_awake = (now - last_loud_time) < KEEP_AWAKE_SEC

            # 💡 [핵심 연산] 2. 깨어있을 때만 무거운 YAMNet 딥러닝 수행
            if is_awake:
                audio_window = np.array(list(audio_buffer)[-WINDOW_SIZE:], dtype=np.float32)
                scores, embeddings, spectrogram = yamnet(audio_window)
                scores_np = scores.numpy()
                S_siren = get_group_score(scores_np, SIREN_CLASSES)
                S_horn = get_group_score(scores_np, HORN_CLASSES)
            else:
                # 자고 있을 때는 연산을 아예 건너뛰고 확률을 0으로 처리 (CPU 휴식)
                S_siren = 0.0
                S_horn = 0.0

            # 나머지 로직은 동일하게 진행 (자고 있으면 값들이 0이라 자연스럽게 NORMAL로 복귀)
            siren_active = 1 if S_siren >= TH_SIREN_ACTIVE else 0
            horn_active = 1 if S_horn >= TH_HORN_ACTIVE else 0

            siren_active_history.append(siren_active)
            horn_active_history.append(horn_active)
            siren_score_history.append(S_siren)
            horn_score_history.append(S_horn)
            rms_db_history.append(latest_rms_db)
            drms_db_history.append(latest_drms_db)

            C_siren = calc_confidence(siren_active_history)
            C_horn = calc_confidence(horn_active_history)
            rms_slope = calc_rms_slope_db_per_window()
            rms_rise = calc_rms_total_rise_db()

            state, level, action = update_state_with_hysteresis(
                S_siren, S_horn, latest_rms_db, latest_drms_db, C_siren, C_horn, rms_slope, rms_rise
            )

            state_history.append(state)
            smooth_state = majority_state()

            # 터미널 출력 제어 (깨어있을 때는 실시간 출력, 잘 때는 3초마다 1번만 Zzz 출력)
            if is_awake:
                print("\n======================================")
                print("🔥 [YAMNet 가동 중] 상태:", state)
                print(f"R_db={latest_rms_db:.2f}, S_siren={S_siren:.3f}, S_horn={S_horn:.3f}")
                print("======================================")
            else:
                if now - sleep_print_time > 3.0:
                    print(f"💤 [휴면 모드] 대기 중... (현재 소리: {latest_rms_db:.2f} dB)")
                    sleep_print_time = now

            send_command(level)

        time.sleep(0.005)

except KeyboardInterrupt:
    print("\n시스템 종료")
    stream.stop()
    stream.close()
    ser.close()
