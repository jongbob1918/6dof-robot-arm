import ikpy.chain
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# --- 1, 2, 3, 4번 부분은 이전과 동일 ---
# --- 1. 로봇 모델 불러오기 ---
script_dir = Path(__file__).parent
urdf_file_path = script_dir / "my_robot.urdf"
my_robot_chain = ikpy.chain.Chain.from_urdf_file(
    urdf_file_path,
    active_links_mask=[False, True, True, True, True, True, False]
)

# --- '준비 위치' 계산 ---
home_position = [0.15, 0, 0.25]
initial_angles_rad = my_robot_chain.inverse_kinematics(home_position)
print(f"✅ '준비 위치' ({home_position}) 자세 계산 완료.")

# --- 2. 사용자가 원하는 최종 목표 지점 설정 ---
desired_target = np.array([0.10, 0.0, .40])
print(f"\n원하는 최종 목표 지점: {desired_target}")

# --- 3. '자동 후퇴 탐색' 로직 ---
current_target = np.copy(desired_target)
final_angles_rad = None
max_iterations = 50
for i in range(max_iterations):
    try:
        calculated_angles = my_robot_chain.inverse_kinematics(
            current_target,
            initial_position=initial_angles_rad
        )
        wrist_pitch_angle = np.rad2deg(calculated_angles[4])
        if not (45 < wrist_pitch_angle < 135):
            raise ValueError(f"손목 각도({wrist_pitch_angle:.1f}도)가 한계를 벗어남")
        final_angles_rad = calculated_angles
        print(f"✅ 자동 탐색 성공! 도달 가능한 목표: {current_target}")
        break
    except Exception as e:
        direction_vector = current_target / np.linalg.norm(current_target)
        current_target -= direction_vector * 0.01
        print(f"  -> 시도 {i+1}: 실패. {current_target} 지점으로 후퇴하여 재시도...")

# --- 최종 결과 확인 ---
if final_angles_rad is None:
    print("\n❌ 자동 탐색 실패: 주변에서 도달 가능한 지점을 찾을 수 없습니다.")
else:
    # --- 5. 시각화 (그리기 순서 변경) ---
    fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
    
    # --- 5-1. 로봇 팔 먼저 모두 그리기 ---
    my_robot_chain.plot(initial_angles_rad, ax)
    my_robot_chain.plot(final_angles_rad, ax)

    # --- 5-2. 그 다음에 글자와 숫자, 경로를 추가 ---
    def plot_joint_numbers(angles_rad, text_prefix, color):
        all_frames = my_robot_chain.forward_kinematics(angles_rad, full_kinematics=True)
        for i, frame_matrix in enumerate(all_frames[1:7]):
            if i < 5:
                position = frame_matrix[:3, 3]
                ax.text(position[0], position[1], position[2], f"  {text_prefix}{i+1}", color=color, fontsize=10, fontweight='bold')

    # 시작/최종 자세에 번호 표시
    plot_joint_numbers(initial_angles_rad, "S", "blue")
    plot_joint_numbers(final_angles_rad, "F", "red")
    
    # 나머지 시각화 요소
    ax.text(home_position[0], home_position[1], home_position[2], "  Start", color='blue')
    ax.text(current_target[0], current_target[1], current_target[2], "  Final", color='green')
    ax.scatter(desired_target[0], desired_target[1], desired_target[2], c='red', marker='x', s=100, label='Desired')
    path_x = [home_position[0], current_target[0]]
    path_y = [home_position[1], current_target[1]]
    path_z = [home_position[2], current_target[2]]
    ax.plot(path_x, path_y, path_z, 'r--')
    
    ax.legend(['End-Effector Path', 'Desired'])
    ax.set_title("Robot Arm: Auto-Search IK Result")
    ax.set_xlim([-0.5, 0.5]); ax.set_ylim([-0.5, 0.5]); ax.set_zlim([0, 0.6])
    ax.set_xlabel("X-axis (m)"); ax.set_ylabel("Y-axis (m)"); ax.set_zlabel("Z-axis (m)")
    
    print("\n3D 그래프 창을 닫으면 프로그램이 종료됩니다.")
    plt.show()