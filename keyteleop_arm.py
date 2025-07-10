import ikpy.chain
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from matplotlib.animation import FuncAnimation

# --- 1. 로봇 모델 불러오기 ---
script_dir = Path(__file__).parent
urdf_file_path = script_dir / "my_robot.urdf"
my_robot_chain = ikpy.chain.Chain.from_urdf_file(
    urdf_file_path,
    active_links_mask=[False, True, True, True, True, True, False]
)

# --- '준비 위치' 계산 ---
home_position = [0.05, 0, 0.20 ]
initial_angles_rad = my_robot_chain.inverse_kinematics(home_position)
print(f"✅ '준비 위치' ({home_position}) 자세 계산 완료.")

# --- 2. 사용자가 원하는 최종 목표 지점 설정 ---
desired_target = np.array([0.30, 0.10, 0.30])
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
        # print(f"  -> 시도 {i+1}: 실패. {current_target} 지점으로 후퇴하여 재시도...")

# --- 최종 결과 확인 ---
if final_angles_rad is None:
    print("\n❌ 자동 탐색 실패: 주변에서 도달 가능한 지점을 찾을 수 없습니다.")
else:
    # --- 5. 애니메이션 시각화 ---
    fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
    
    # 애니메이션 프레임 수 설정
    num_frames = 50

    # 시작 각도와 최종 각도 사이를 보간하여 중간 각도들 생성
    all_angles = np.linspace(initial_angles_rad, final_angles_rad, num=num_frames)

    # 애니메이션의 각 프레임을 업데이트하는 함수
    def animate(frame_index):
        # 이전 프레임의 내용을 지움
        ax.clear()

        # 현재 프레임의 각도로 로봇 팔을 그림
        my_robot_chain.plot(all_angles[frame_index], ax)
        
        # 그래프 배경 및 라벨 등 고정된 요소들을 매번 다시 그려줌
        ax.set_title(f"Robot Arm Animation (Frame {frame_index+1}/{num_frames})")
        ax.set_xlim([-0.5, 0.5]); ax.set_ylim([-0.5, 0.5]); ax.set_zlim([0, 0.6])
        ax.set_xlabel("X-axis (m)"); ax.set_ylabel("Y-axis (m)"); ax.set_zlabel("Z-axis (m)")
        
        # 시작/목표 지점 텍스트 및 마커 표시
        ax.text(home_position[0], home_position[1], home_position[2], "  Start", color='blue')
        ax.text(current_target[0], current_target[1], current_target[2], "  Final", color='green')
        ax.scatter(desired_target[0], desired_target[1], desired_target[2], c='red', marker='x', s=100)

    # FuncAnimation 객체를 생성하여 애니메이션 실행
    # interval: 프레임 간 간격 (밀리초)
    ani = FuncAnimation(fig, animate, frames=num_frames, interval=50)

    print("\n애니메이션을 재생합니다. 그래프 창을 닫으면 프로그램이 종료됩니다.")
    plt.show()