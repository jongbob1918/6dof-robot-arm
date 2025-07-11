#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include "6dof-kinematic.h" // 기존 클래스 헤더

// 8개의 해 중에서 현재 각도와 가장 차이가 적은 최적의 해를 선택하는 함수
DOF6Kinematic::Joint6D_t findBestSolution(const DOF6Kinematic::IKSolves_t& solves, const DOF6Kinematic::Joint6D_t& lastJoints) {
    DOF6Kinematic::Joint6D_t bestSolution = solves.config[0];
    float minDiff = INFINITY;

    for (int i = 0; i < 8; ++i) {
        // solFlag[i][0], [i][1], [i][2]가 모두 1 이상(유효한 해)인지 확인
        if (solves.solFlag[i][0] > 0 && solves.solFlag[i][1] > 0 && solves.solFlag[i][2] > 0) {
            float currentDiff = 0;
            for (int j = 0; j < 6; ++j) {
                currentDiff += pow(solves.config[i].a[j] - lastJoints.a[j], 2);
            }

            if (currentDiff < minDiff) {
                minDiff = currentDiff;
                bestSolution = solves.config[i];
            }
        }
    }
    return bestSolution;
}

int main(int argc, char *argv[]) {
    // 1. 명령줄 인자 확인 (프로그램 이름 + 12개 인자 = 13개)
    if (argc != 13) {
        std::cerr << "Usage: " << argv[0] << " pX pY pZ oA oB oC lastA1 lastA2 lastA3 lastA4 lastA5 lastA6" << std::endl;
        return 1;
    }

    // 2. 로봇 팔 치수 정의 (실제 로봇에 맞게 수정 필요)
    // DOF6Kinematic(L_BS, D_BS, L_AM, L_FA, D_EW, L_WT);
    DOF6Kinematic kinematicSolver(35.0, 0.0, 105.0, 105.0, 0.0, 35.0);

    // 3. 입력 값 파싱
    // 목표 자세
    DOF6Kinematic::Pose6D_t targetPose(
        std::stof(argv[1]), std::stof(argv[2]), std::stof(argv[3]),
        std::stof(argv[4]), std::stof(argv[5]), std::stof(argv[6])
    );
    // 마지막 관절 각도 (부드러운 움직임을 위해)
    DOF6Kinematic::Joint6D_t lastJoints(
        std::stof(argv[7]), std::stof(argv[8]), std::stof(argv[9]),
        std::stof(argv[10]), std::stof(argv[11]), std::stof(argv[12])
    );

    // 4. IK 계산
    DOF6Kinematic::IKSolves_t solutions;
    if (kinematicSolver.SolveIK(targetPose, lastJoints, solutions)) {
        // 5. 최적의 해 선택
        DOF6Kinematic::Joint6D_t bestSol = findBestSolution(solutions, lastJoints);

        // 6. 결과 출력 (Python에서 파싱하기 쉬운 형태로)
        std::cout << "angles:";
        for (int i = 0; i < 6; ++i) {
            std::cout << " " << bestSol.a[i];
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Error: IK solution not found." << std::endl;
        return 1;
    }

    return 0;
}