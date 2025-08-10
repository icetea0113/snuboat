import socket
import time
import random

def nmea_checksum(sentence: str) -> str:
    """$와 * 사이의 문자열에 대한 NMEA 체크섬 계산"""
    if sentence.startswith('$'):
        sentence = sentence[1:]
    if '*' in sentence:
        sentence = sentence.split('*')[0]
    csum = 0
    for ch in sentence:
        csum ^= ord(ch)
    return f"{csum:02X}"

# 송신 대상 설정
UDP_IP = "127.0.0.1"     # 수신할 노드 IP
UDP_PORT = 56001         # 수신할 노드 포트
SEND_INTERVAL = 0.5      # 전송 간격 (초)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    while True:
        # 랜덤 값 생성 (기본값 ±범위)
        x = 5.0 + random.uniform(-0.5, 0.5)     # m
        y = 4.0 + random.uniform(-0.5, 0.5)     # m
        z = 3.0 + random.uniform(-0.2, 0.2)     # m
        roll = 1.0 + random.uniform(-0.1, 0.1)  # deg
        pitch = 2.0 + random.uniform(-0.1, 0.1) # deg
        speed = 3.0 + random.uniform(-0.2, 0.2) # m/s
        cog = 90.0 + random.uniform(-1.0, 1.0)  # deg

        # 본문 데이터 구성
        msg_body = f"$CURMC,015352.00,A,{x:.3f},{y:.3f},{z:.3f},{roll:.3f},{pitch:.3f},{speed:.3f},0.0,0.0,{cog:.3f},062822,0.0"

        # 체크섬 계산 후 최종 메시지
        checksum = nmea_checksum(msg_body)
        full_msg = f"{msg_body}*{checksum}"

        # UDP 전송
        sock.sendto(full_msg.encode('ascii'), (UDP_IP, UDP_PORT))
        print(f"Sent: {full_msg}")

        time.sleep(SEND_INTERVAL)

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    sock.close()
