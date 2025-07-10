import ikpy.chain
import numpy as np
import socket # 통신 라이브러리
import time
from pathlib import Path
from pynput import keyboard

# --- 1. 통신 설정 ---
# 아두이노 시리얼 모니터에 출력된 ESP32의 IP 주소
ESP32_IP = "192.168.0.2" # <-- 사용자의 ESP32 IP 주소로 수정!
TCP_PORT = 23

# TCP 소켓 생성 및 서버에 연결
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ESP32_IP, TCP_PORT))
    # 아래 라인의 끝에 있던 `` 문자를 제거했습니다.
    print(f"✅ TCP 서버({ESP32_IP}:{TCP_PORT})에 성공적으로 연결되었습니다.")
except Exception as e:
    print(f"❌ TCP 서버 연결 실패: {e}")
    exit()


# --- 2. 로봇 모델 및 초기화 ---
script_dir = Path(__file__).parent
urdf_file_path = script_dir / "my_robot.urdf"
my_robot_chain = ikpy.chain.Chain.from_urdf_file(
    urdf_file_path,
    active_links_mask=[False, True, True, True, True, True, False]
)
neutral_pose_rad = np.deg2rad([0, 90, 90, 90, 90, 90, 0])
home_position = [0.15, 0, 0.2]
current_angles_rad = my_robot_chain.inverse_kinematics(home_position, initial_position=neutral_pose_rad)
desired_target = np.array(home_position)
step = 0.01

# --- 3. 키보드 리스너 설정 ---
key_pressed = set()
def on_press(key): key_pressed.add(key)
def on_release(key):
    if key in key_pressed: key_pressed.remove(key)
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("\n키보드로 로봇 팔을 조종하세요.")
print("방향키 위/아래: X-axis (+/-) | 방향키 좌/우: Y-axis (-/+) | R/F: Z-axis (+/-)")
print("ESC 키를 누르면 종료됩니다.")

# --- 4. 메인 제어 루프 ---
running = True
while running:
    original_target = np.copy(desired_target)
    
    if keyboard.Key.esc in key_pressed: running = False; break
    if keyboard.Key.up in key_pressed: desired_target[0] += step
    if keyboard.Key.down in key_pressed: desired_target[0] -= step
    if keyboard.Key.right in key_pressed: desired_target[1] += step
    if keyboard.Key.left in key_pressed: desired_target[1] -= step
    if keyboard.KeyCode.from_char('r') in key_pressed: desired_target[2] += step
    if keyboard.KeyCode.from_char('f') in key_pressed: desired_target[2] -= step

    # --- 5. IK 계산 및 TCP 전송 ---
    try:
        calculated_angles = my_robot_chain.inverse_kinematics(
            desired_target,
            initial_position=current_angles_rad
        )
        current_angles_rad = calculated_angles
        angles_deg = np.rad2deg(current_angles_rad)
        
        command_parts = []
        for i in range(1, 6):
            angle = int(np.clip(angles_deg[i], 10, 170))
            command_parts.append(f"A{i}:{angle}")
        
        command = ",".join(command_parts) + "\n"
        
        sock.sendall(command.encode('utf-8'))
        print(f"전송: {command.strip()}", end='\r')
        
    except Exception:
        desired_target = original_target
    
    time.sleep(0.05)

# --- 6. 종료 ---
listener.stop()
sock.close()
print("\n프로그램을 종료합니다.")