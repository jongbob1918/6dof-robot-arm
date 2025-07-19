import numpy as np
import socket
import time
import subprocess
from pynput import keyboard
import pyvista as pv
import threading

# --- 1. 로봇 사양 및 설정 ---
L_BASE = 0.088
L_ARM = 0.104
L_FOREARM = 0.125
L_WRIST = 0.162
ESP32_IP = "192.168.0.7"
TCP_PORT = 23
IK_SOLVER_PATH = "./ik_solver"
sock = None
is_robot_connected = False

# --- 2. 전역 변수 ---
# 초기 각도 및 위치 계산
current_angles_deg = np.array([90.0, 90.0, 180.0, 0.0, 0.0, 0.0])
# FK 함수는 시작 시 위치 계산에만 사용되므로 여기에 한 번만 정의
q_init = np.deg2rad(current_angles_deg)
dh_params_init = [
    [q_init[0], L_BASE, 0.0, -np.pi/2], [q_init[1] - np.pi/2, 0.0, L_ARM, 0.0],
    [q_init[2], 0.0, 0.0, -np.pi/2], [q_init[3], L_FOREARM, 0.0, np.pi/2],
    [q_init[4], 0.0, 0.0, -np.pi/2], [q_init[5], L_WRIST, 0.0, 0.0]
]
T_init = np.identity(4)
for theta, d, a, alpha in dh_params_init:
    A = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                  [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                  [0, np.sin(alpha), np.cos(alpha), d], [0, 0, 0, 1]])
    T_init = T_init @ A
desired_position = T_init[:3, 3] * 1000

target_orientation_euler = [0.0, 0.0, 0.0]
step = 2.0
key_pressed = set()
running = True

# --- 3. 핵심 함수들 ---
def connect_to_robot():
    global sock
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        sock.connect((ESP32_IP, TCP_PORT))
        print(f"✅ TCP 서버({ESP32_IP}:{TCP_PORT})에 성공적으로 연결되었습니다.")
        return True
    except Exception as e:
        print(f"⚠️ TCP 서버 연결 실패: {e}")
        return False

def get_forward_kinematics(joint_angles_deg):
    q = np.deg2rad(joint_angles_deg)
    dh_params = [
        [q[0], L_BASE, 0.0, -np.pi/2], [q[1] - np.pi/2, 0.0, L_ARM, 0.0],
        [q[2], 0.0, 0.0, -np.pi/2], [q[3], L_FOREARM, 0.0, np.pi/2],
        [q[4], 0.0, 0.0, -np.pi/2], [q[5], L_WRIST, 0.0, 0.0]
    ]
    transforms = [np.identity(4)]
    T = np.identity(4)
    for theta, d, a, alpha in dh_params:
        A = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                      [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                      [0, np.sin(alpha), np.cos(alpha), d], [0, 0, 0, 1]])
        T = T @ A
        transforms.append(T)
    return np.array([T[:3, 3] for T in transforms]) * 1000

def on_press(key):
    key_pressed.add(key)

def on_release(key):
    global running
    if key in key_pressed:
        key_pressed.remove(key)
    if key == keyboard.Key.esc:
        running = False
        return False

def control_loop():
    global desired_position, current_angles_deg
    while running:
        original_position = np.copy(desired_position)
        moved = False
        if keyboard.Key.up in key_pressed: desired_position[0] += step; moved = True
        if keyboard.Key.down in key_pressed: desired_position[0] -= step; moved = True
        if keyboard.Key.right in key_pressed: desired_position[1] += step; moved = True
        if keyboard.Key.left in key_pressed: desired_position[1] -= step; moved = True
        if keyboard.KeyCode.from_char('r') in key_pressed: desired_position[2] += step; moved = True
        if keyboard.KeyCode.from_char('f') in key_pressed: desired_position[2] -= step; moved = True
        
        if moved:
            try:
                cmd = [IK_SOLVER_PATH] + list(map(str, desired_position)) + \
                      list(map(str, target_orientation_euler)) + \
                      list(map(str, current_angles_deg))

                result = subprocess.run(cmd, capture_output=True, text=True, check=True, timeout=0.5)
                output = result.stdout.strip()

                if output.startswith("angles:"):
                    parts = output.split()
                    current_angles_deg = np.array([float(p) for p in parts[1:]])
                    if is_robot_connected:
                        servo_angles = current_angles_deg[1:6].copy()
                        servo_angles[2] = 180 - servo_angles[2]
                        command_parts = [f"A{i+1}:{int(np.clip(angle, 10, 170))}" for i, angle in enumerate(servo_angles)]
                        command = ",".join(command_parts) + "\n"
                        sock.sendall(command.encode('utf-8'))
                else:
                    desired_position = original_position
            except Exception:
                desired_position = original_position
        time.sleep(0.05)

# --- 4. 메인 실행 ---
if __name__ == "__main__":
    is_robot_connected = connect_to_robot()
    
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()

    # PyVista Plotter 설정
    plotter = pv.Plotter(window_size=[960, 720])
    plotter.add_axes()
    plotter.camera_position = 'iso'
    plotter.camera.zoom(1.5)
    
    print("3D 시각화 창을 시작합니다. 창을 클릭 후 키보드로 조종하세요.")
    print("ESC 키를 누르면 종료됩니다.")
    plotter.show(interactive_update=True)
    # 메인 렌더링 루프
    while running:
        # 현재 각도를 기반으로 로봇 팔 위치 계산
        joint_positions = get_forward_kinematics(current_angles_deg)
        
        # 화면 지우기
        plotter.clear()
        
        # 로봇 팔과 관절 다시 그리기
        plotter.add_lines(joint_positions, width=10, connected=True)
        plotter.add_points(joint_positions, render_points_as_spheres=True, point_size=15, color='red')
        
        # 각도 정보를 텍스트로 만들어 화면에 추가
        angle_text = "--- Joint Angles ---\n" + \
                     "\n".join([f"  Joint {i+1}: {angle:.1f}°" for i, angle in enumerate(current_angles_deg)])
        plotter.add_text(
            angle_text, 
            position='upper_right', # 위치를 오른쪽 위로 변경
            font_size=15,           # 폰트 크기를 키움
            color='yellow',         # 색상을 눈에 띄는 노란색으로 변경
            name='angle_display'    # 텍스트 객체에 이름 부여 (업데이트 시 식별용)
        )
        
        # 화면 렌더링
        plotter.update()
        time.sleep(0.03) # 렌더링 루프 간격

    # 종료 처리
    listener.stop()
    if is_robot_connected:
        sock.close()
    plotter.close()
    print("\n프로그램을 종료합니다.")