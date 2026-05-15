import numpy as np
import tflite_runtime.interpreter as tflite
import serial
import time

# 모델 로드
interpreter = tflite.Interpreter(model_path="models/pi_model.tflite")
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# UART 포트 설정 
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# 위험도 산출 함수
def calculate_risk_score(S, R, D, C):
    return (0.785 * S) + (0.140 * R) + (0.045 * D) + (0.030 * C)

print("시스템 구동 시작 (UART 대기 중입니다...)\n")

while True:
    try:
        # ESP32 수신 응답 읽기
        if ser.in_waiting > 0:
            esp_msg = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # 1차 필터 통과 신호를 받았을 때
            if "TRIGGER" in esp_msg:
                print("\n(ESP32 -> Pi) TRIGGER 수신 (YAMNet 분석 시작)")
                
                # [TODO] 나중에 실제 수신된 오디오 데이터로 교체
                dummy_audio = np.zeros(input_details[0]['shape'], dtype=np.float32)
                
                # 모델 추론
                interpreter.set_tensor(input_details[0]['index'], dummy_audio)
                interpreter.invoke()
                scores = interpreter.get_tensor(output_details[0]['index'])
                
                # 테스트용 가상 변수
                S_val = 0.85 
                R_val = 0.50 
                D_val = 0.20
                C_val = 0.90
                
                final_risk = calculate_risk_score(S_val, R_val, D_val, C_val)
                print(f"(Pi 내부) 위험도 계산 완료: {final_risk:.3f}")
                
                # 결과 전송
                if final_risk >= 0.3:
                    print("(Pi -> ESP) DUCKING (위험 감지)")
                    ser.write(b"DUCKING\n")
                else:
                    print("(Pi -> ESP) NORMAL (상황 종료)")
                    ser.write(b"NORMAL\n")
                    
            else:
                # 일반 로그 수신
                print(f"  └─ (ESP32 -> Pi) {esp_msg}")
                
        time.sleep(0.01)
        
    except KeyboardInterrupt:
        print("\n시스템 종료")
        ser.close()
        break
