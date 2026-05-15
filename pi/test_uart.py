import serial
import time

# UART 포트 설정 
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

print("UART 통신 테스트 시작\n")

while True:
    # 1. DUCKING 명령 송신
    print("(Pi -> ESP) DUCKING (사이렌 상황 가정)")
    ser.write(b"DUCKING\n")
    
    # ESP32의 수신 응답 읽기
    time.sleep(0.5) # 수신을 받기 전 딜레이
    while ser.in_waiting > 0:
        esp_response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  └─ (ESP32 -> Pi) {esp_response}")
        
    time.sleep(4.5)

    # 2. NORMAL 송신받기
    print("\n(Pi -> ESP) NORMAL (상황 종료 가정)")
    ser.write(b"NORMAL\n")
    
    # ESP32의 수신 응답 읽기
    time.sleep(0.5)
    while ser.in_waiting > 0:
        esp_response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  └─ (ESP32 -> Pi) {esp_response}")
        
    time.sleep(4.5)
    print("-" * 40)
