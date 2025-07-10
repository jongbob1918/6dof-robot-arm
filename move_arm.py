import ikpy.chain
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from pynput import keyboard

# --- 1. 로봇 모델 불러오기 ---
script_dir = Path(__file__).parent
urdf_file_path = script_dir / "my_robot.urdf"
my_robot_chain = ikpy.chain.Chain.from_urdf_file(
    urdf_file_path,
    active_links_mask=[False, True, True, True, True, True, False]
)

# --- 2. 변수 초기화 ---
# 중립 자세를 IK 계산의 초기 추측값으로 사용
neutral_pose_rad = np.deg2rad([0, 90, 90, 90, 90, 90, 0])
# 시작 위치 및 현재 로봇 각도
home_position = [0.15, 0, 0.2]
current_angles_rad = my_robot_chain.inverse_kinematics(home_position, initial_position=neutral_pose_rad)
# 사용자가 키보드로 조종할 목표 지점 (시작은 home_position)
desired_target = np.array(home_position)
# 키보드 조종 시 이동할 거리 (1cm)
step = 0.01

# --- 3. 3D 그래프 및 키보드 리스너 설정 ---
plt.ion() # 대화형 모드 켜기
fig, ax = plt.subplots(subplot_kw={'projection': '3d'})

key_pressed = set()
def on_press(key):
    key_pressed.add(key)
def on_release(key):
    if key in key_pressed:
        key_pressed.remove(key)
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("키보드로 로봇 팔의 목표 지점을 조종하세요.")
print("W/S: Y-axis (+/-), A/D: X-axis (-/+), Q/E: Z-axis (+/-)")
print("ESC 키를 누르면 종료됩니다.")

# --- 4. 메인 제어 루프 ---
running = True
while running:
    # 키보드 입력에 따라 목표 지점 업데이트
    if keyboard.Key.esc in key_pressed:
        running = False
        break
    if keyboard.KeyCode.from_char('d') in key_pressed: desired_target[0] += step
    if keyboard.KeyCode.from_char('a') in key_pressed: desired_target[0] -= step
    if keyboard.KeyCode.from_char('w') in key_pressed: desired_target[1] += step
    if keyboard.KeyCode.from_char('s') in key_pressed: desired_target[1] -= step
    if keyboard.KeyCode.from_char('q') in key_pressed: desired_target[2] += step
    if keyboard.KeyCode.from_char('e') in key_pressed: desired_target[2] -= step

    # --- 5. IK 계산 및 그래프 업데이트 ---
    try:
        # 현재 목표 지점에 대한 IK 계산
        calculated_angles = my_robot_chain.inverse_kinematics(
            desired_target,
            initial_position=current_angles_rad # 현재 자세에서 가장 가까운 해를 찾음
        )
        # 계산 성공 시, 현재 로봇 각도 업데이트
        current_angles_rad = calculated_angles

        # 그래프 업데이트
        ax.clear()
        my_robot_chain.plot(current_angles_rad, ax)
        ax.scatter(desired_target[0], desired_target[1], desired_target[2], c='red', marker='x', s=100)
        ax.text(desired_target[0], desired_target[1], desired_target[2], "  Target", color='red')
        ax.set_title("Real-time Robot Arm Control")
        ax.set_xlim([-0.5, 0.5]); ax.set_ylim([-0.5, 0.5]); ax.set_zlim([0, 0.6])
        ax.set_xlabel("X-axis (m)"); ax.set_ylabel("Y-axis (m)"); ax.set_zlabel("Z-axis (m)")
        
        plt.pause(0.01) # 0.01초 대기하며 그래프 갱신

    except Exception:
        # IK 계산 실패 시 (도달 불가 등), 목표 지점을 이전 위치로 되돌림
        print("도달할 수 없는 위치입니다.")
        # 키보드 입력 취소
        if keyboard.KeyCode.from_char('d') in key_pressed: desired_target[0] -= step
        if keyboard.KeyCode.from_char('a') in key_pressed: desired_target[0] += step
        if keyboard.KeyCode.from_char('w') in key_pressed: desired_target[1] -= step
        if keyboard.KeyCode.from_char('s') in key_pressed: desired_target[1] += step
        if keyboard.KeyCode.from_char('q') in key_pressed: desired_target[2] -= step
        if keyboard.KeyCode.from_char('e') in key_pressed: desired_target[2] += step
        plt.pause(0.01)

# --- 6. 종료 ---
listener.stop()
plt.ioff()
print("프로그램을 종료합니다.")
plt.show() # 마지막 모습을 보여주기 위해 창을 유지