#include <ESP32Servo.h>
#include <math.h>

// --- 하드웨어 정의 ---
#define MOTOR_PIN_1_BASE_YAW      (23)   // PWM 지원 핀
#define MOTOR_PIN_2_SHOULDER_PITCH (22)   // PWM 지원 핀
#define MOTOR_PIN_3_ELBOW_PITCH   (21)  // PWM 지원 핀
#define MOTOR_PIN_4_WRIST_PITCH   (19)   // PWM 지원 핀
#define MOTOR_PIN_5_WRIST_ROLL    (18)   // PWM 지원 핀
#define MOTOR_PIN_TONGS           (17)  // PWM 지원 핀

// --- 상수 ---
#define DEGREE_MIN    (0)
#define DEGREE_MAX    (180)
#define NOMOV         (255) // 모터 움직임 없음을 나타내는 값
#define MAX_MOTORS    (6)   // 최대 모터 수 (5축 + 그리퍼)

// --- 기본 위치 ---
#define DEFAULT_DEGREE_M1 (90)
#define DEFAULT_DEGREE_M2 (130)
#define DEFAULT_DEGREE_M3 (150)
#define DEFAULT_DEGREE_M4 (50)
#define DEFAULT_DEGREE_M5 (90)
#define DEFAULT_DEGREE_TONGS (80)

// 전방 선언
class Motor;

// 모터 이동 명령을 정의하는 구조체
struct MotorCommand 
{
  Servo* servo;
  int targetDegree;
  bool invertRotation;
};

class Motor 
{
  private:
    // 개별 모터 속도 프로파일 관리를 위한 내부 클래스
    class SpeedControlParam 
    {
      public:
        int fromDegree;
        int toDegree;
        int currDegree;
        int delta;
        int momentaryDelay;
        bool isActive; // 활성 상태 플래그 추가

        SpeedControlParam() : isActive(false) {} // 기본 생성자

        SpeedControlParam(int fromDegree, int toDegree) 
        {
          if (toDegree == NOMOV || fromDegree == toDegree) 
          {
            this->currDegree = NOMOV;
            this->toDegree = NOMOV;
            this->momentaryDelay = 0; // 명시적 초기화
            this->isActive = false;
            return;
          }
          
          this->fromDegree = fromDegree;
          this->toDegree = toDegree;
          this->delta = (fromDegree < toDegree) ? 1 : -1; 
          this->currDegree = fromDegree + this->delta;
          this->momentaryDelay = -1;
          this->isActive = true;

          checkLimit(&this->toDegree);
        }

        void checkLimit(int *degree) 
        {
            if (*degree > DEGREE_MAX) 
            {
                *degree = DEGREE_MAX;
            }
            else if (*degree < DEGREE_MIN) 
            {
              *degree = DEGREE_MIN;
            }
        }
    };
  
  protected:
    // 5개 축 + 집게를 위한 정적 서보 객체 (스택 할당으로 변경)
    static Servo motor1_base_yaw;
    static Servo motor2_shoulder_pitch;
    static Servo motor3_elbow_pitch;
    static Servo motor4_wrist_pitch;
    static Servo motor5_wrist_roll;
    static Servo tongsMotor;  
    static bool initialized; // 초기화 상태 플래그

    void initialize() 
    {
        if (initialized) return; // 중복 초기화 방지
        
        Serial.println("[i] Initializing motor system...");
        
        // ESP32Servo 라이브러리 초기화 (타이머 할당)
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);
        
        // 서보 attach 시 PWM 주파수 설정 (50Hz) 및 펄스 폭 범위 설정
        motor1_base_yaw.setPeriodHertz(50);
        motor2_shoulder_pitch.setPeriodHertz(50);
        motor3_elbow_pitch.setPeriodHertz(50);
        motor4_wrist_pitch.setPeriodHertz(50);
        motor5_wrist_roll.setPeriodHertz(50);
        tongsMotor.setPeriodHertz(50);
        
        // 서보 연결 (500-2500μs 펄스 폭으로 MG996R 등 표준 서보 호환)
        motor1_base_yaw.attach(MOTOR_PIN_1_BASE_YAW, 500, 2500);
        motor2_shoulder_pitch.attach(MOTOR_PIN_2_SHOULDER_PITCH, 500, 2500);
        motor3_elbow_pitch.attach(MOTOR_PIN_3_ELBOW_PITCH, 500, 2500);
        motor4_wrist_pitch.attach(MOTOR_PIN_4_WRIST_PITCH, 500, 2500);
        motor5_wrist_roll.attach(MOTOR_PIN_5_WRIST_ROLL, 500, 2500);
        tongsMotor.attach(MOTOR_PIN_TONGS, 500, 2500);
        
        initialized = true;
        Serial.println("[i] Motor system initialized successfully.");
    }

    // 개선된 가우시안 속도 프로파일 함수 (최소 지연값 증가)
    int getMomentaryDelayByGauss(int fromDegree, int toDegree, int currDegree) 
    {
      if (fromDegree == toDegree) 
      {
        return 0;
      }
      
      // 경계 조건 확인 (기존 로직 유지)
      if (fromDegree < toDegree) 
      {
        currDegree = (currDegree > toDegree) ? toDegree : currDegree;
        currDegree = (currDegree < fromDegree) ? fromDegree : currDegree;
      }
      else 
      {
        currDegree = (currDegree > fromDegree) ? fromDegree : currDegree;
        currDegree = (currDegree < toDegree) ? toDegree : currDegree;
      }
    
      float x = ((float)currDegree - fromDegree) / (toDegree - fromDegree);

      // 전류 스파이크 방지를 위한 최소 지연값 증가 (5ms → 8ms)
      const float MOMENTARY_DELAY_MS_MAX = 25.0; // 최대값도 약간 증가
      const float MOMENTARY_DELAY_MS_MIN = 8.0;  // 최소값 증가로 안정성 향상
      const float MOMENTARY_DELAY_MS_DIFF = MOMENTARY_DELAY_MS_MAX - MOMENTARY_DELAY_MS_MIN;
      const float ROTATE_DEGREE_MAX = 180.0;
      const float B = 0.5;
      const float C = 0.235;
      
      float y = MOMENTARY_DELAY_MS_MAX - MOMENTARY_DELAY_MS_DIFF * exp(-((x - B) * (x - B)) / (C * C)) * (abs(fromDegree - toDegree) / ROTATE_DEGREE_MAX);
      
      return round(y);
    }    

    // 생성자
    Motor() 
    {
        initialize();
    }

    // 개선된 모션 엔진 (VLA 제거, 오버슈트 가드 개선)
    void moveMotorsConcurrently(MotorCommand commands[], int numMotors) 
    {
        if (!initialized) {
            Serial.println("[E] Motors not initialized!");
            return;
        }
        
        if (numMotors > MAX_MOTORS) {
            Serial.println("[E] Too many motors specified!");
            return;
        }
        
        // 유효성 검사
        for (int i = 0; i < numMotors; i++) {
            if (commands[i].servo == nullptr) {
                Serial.println("[E] Invalid servo pointer!");
                return;
            }
        }
        
        // VLA 제거: 고정 크기 배열 사용
        SpeedControlParam params[MAX_MOTORS];
        bool allMovesComplete = false;
        int activeMotors = 0;

        // 초기 파라미터 설정
        for (int i = 0; i < numMotors; i++) 
        {
            int startDegree = commands[i].servo->read();
            int targetDegree = commands[i].targetDegree;

            // 반전 로직 적용
            if (commands[i].invertRotation) 
            {
                targetDegree = DEGREE_MAX - targetDegree;
            }
            
            // 범위 제한
            if (targetDegree > DEGREE_MAX) targetDegree = DEGREE_MAX;
            if (targetDegree < DEGREE_MIN) targetDegree = DEGREE_MIN;
            
            params[i] = SpeedControlParam(startDegree, targetDegree);
            if (params[i].isActive) {
                activeMotors++;
            }
        }

        if (activeMotors == 0) {
            Serial.println("[i] No motors need to move.");
            return;
        }

        Serial.print("[i] Moving ");
        Serial.print(activeMotors);
        Serial.println(" motors concurrently...");

        // 주 제어 루프
        unsigned long lastUpdate = millis();
        while (!allMovesComplete) 
        {
            allMovesComplete = true;

            for (int i = 0; i < numMotors; i++) 
            {
                // 비활성 모터나 완료된 모터 건너뛰기
                if (!params[i].isActive || params[i].currDegree == NOMOV || params[i].currDegree == params[i].toDegree) 
                {
                    continue;
                }
                
                allMovesComplete = false;

                // 다음 단계 실행 시간 확인
                if (params[i].momentaryDelay < 0) 
                {
                    commands[i].servo->write(params[i].currDegree);
                    
                    // 가우시안 프로파일로 지연 계산
                    params[i].momentaryDelay = getMomentaryDelayByGauss(params[i].fromDegree, params[i].toDegree, params[i].currDegree);
                    
                    // 다음 각도로 진행
                    params[i].currDegree += params[i].delta;
                    
                    // 개선된 오버슈트 가드
                    if ((params[i].delta > 0 && params[i].currDegree > params[i].toDegree) ||
                        (params[i].delta < 0 && params[i].currDegree < params[i].toDegree)) {
                        params[i].currDegree = params[i].toDegree;
                        commands[i].servo->write(params[i].currDegree);
                    }
                }
            }

            delay(1); // 1ms 시스템 심장박동

            // 지연 카운터 감소
            for (int i = 0; i < numMotors; i++) 
            {
                if (params[i].isActive && params[i].momentaryDelay >= 0) {
                    params[i].momentaryDelay--;
                }
            }

            // 타임아웃 방지 (30초)
            if (millis() - lastUpdate > 30000) {
                Serial.println("[W] Motion timeout - stopping.");
                break;
            }
        }
        
        Serial.println("[i] Motion complete.");
    }
};

// 정적 멤버 초기화 (스택 할당으로 변경)
Servo Motor::motor1_base_yaw;
Servo Motor::motor2_shoulder_pitch;
Servo Motor::motor3_elbow_pitch;
Servo Motor::motor4_wrist_pitch;
Servo Motor::motor5_wrist_roll;
Servo Motor::tongsMotor;
bool Motor::initialized = false;

class Robot : public Motor 
{
  private:
    static Robot *_instance;
 
    Robot() : Motor() {}

  public:
    static Robot* getInstance() 
    {
        if (_instance == nullptr) 
        {
            _instance = new Robot();
        }
        return _instance;
    }
    void primeToDefault() 
    {
        Serial.println("[i] Priming motors to default position...");
        motor1_base_yaw.write(DEFAULT_DEGREE_M1);
        motor2_shoulder_pitch.write(DEFAULT_DEGREE_M2);
        // M3는 회전 방향이 반대이므로, 설정값도 반대로 적용해야 합니다.
        motor3_elbow_pitch.write(DEGREE_MAX - DEFAULT_DEGREE_M3); 
        motor4_wrist_pitch.write(DEFAULT_DEGREE_M4);
        motor5_wrist_roll.write(DEFAULT_DEGREE_M5);
        tongsMotor.write(DEFAULT_DEGREE_TONGS);
        Serial.println("[i] Priming complete.");
    }
    // 5축 팔 제어
    void movArms(int m1_deg, int m2_deg, int m3_deg, int m4_deg, int m5_deg) 
    {
        const int NUM_ARM_MOTORS = 5;
        MotorCommand commands[NUM_ARM_MOTORS] = 
        {
            {&motor1_base_yaw,       m1_deg, false},
            {&motor2_shoulder_pitch, m2_deg, false},
            {&motor3_elbow_pitch,    m3_deg, true},  // 반전 활성화
            {&motor4_wrist_pitch,    m4_deg, false},
            {&motor5_wrist_roll,     m5_deg, false}
        };
        moveMotorsConcurrently(commands, NUM_ARM_MOTORS);
    }

    // 집게 제어
    void movTongs(int toDegree) 
    {
        MotorCommand commands[1] = 
        {
            {&tongsMotor, toDegree, false}
        };
        moveMotorsConcurrently(commands, 1);
    }

    // 기본 위치로 이동
    void movDefault() 
    {
        Serial.println("[i] Moving to default position...");
        movArms(DEFAULT_DEGREE_M1, DEFAULT_DEGREE_M2, DEFAULT_DEGREE_M3, DEFAULT_DEGREE_M4, DEFAULT_DEGREE_M5);
        delay(100); // 팔 이동 완료 대기
        movTongs(DEFAULT_DEGREE_TONGS);
        Serial.println("[i] Default position reached.");
    }
    
    // 현재 위치 출력 (JSON 형태로 개선)
    void printCurrentPosition() 
    {
        Serial.println("{");
        Serial.print("  \"m1_base_yaw\": "); Serial.print(motor1_base_yaw.read()); Serial.println(",");
        Serial.print("  \"m2_shoulder_pitch\": "); Serial.print(motor2_shoulder_pitch.read()); Serial.println(",");
        Serial.print("  \"m3_elbow_pitch\": "); Serial.print(motor3_elbow_pitch.read()); Serial.println(",");
        Serial.print("  \"m4_wrist_pitch\": "); Serial.print(motor4_wrist_pitch.read()); Serial.println(",");
        Serial.print("  \"m5_wrist_roll\": "); Serial.print(motor5_wrist_roll.read()); Serial.println(",");
        Serial.print("  \"tongs\": "); Serial.println(tongsMotor.read());
        Serial.println("}");
    }

    // 개별 모터 제어 (디버깅용)
    void moveMotor(int motorIndex, int degree) 
    {
        if (degree < DEGREE_MIN || degree > DEGREE_MAX) {
            Serial.println("[E] Degree out of range (0-180).");
            return;
        }

        Servo* targetServo = nullptr;
        String motorName = "";
        
        switch (motorIndex) {
            case 1: targetServo = &motor1_base_yaw; motorName = "Base Yaw"; break;
            case 2: targetServo = &motor2_shoulder_pitch; motorName = "Shoulder Pitch"; break;
            case 3: targetServo = &motor3_elbow_pitch; motorName = "Elbow Pitch"; break;
            case 4: targetServo = &motor4_wrist_pitch; motorName = "Wrist Pitch"; break;
            case 5: targetServo = &motor5_wrist_roll; motorName = "Wrist Roll"; break;
            case 6: targetServo = &tongsMotor; motorName = "Tongs"; break;
            default:
                Serial.println("[E] Invalid motor index (1-6).");
                return;
        }

        MotorCommand commands[1] = {{targetServo, degree, (motorIndex == 3)}};
        Serial.print("[i] Moving "); Serial.print(motorName); Serial.print(" to "); Serial.println(degree);
        moveMotorsConcurrently(commands, 1);
    }
};

Robot* Robot::_instance = nullptr;

void setup() 
{
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    
    Serial.println("[i] ESP32 5-Axis Robot Control System v2.0");
    Serial.println("[i] Hardware: 5-DOF Arm + Gripper");
    Serial.println("[i] Initializing...");
    
    // 로봇 인스턴스 생성 및 초기화
    Robot *robot = Robot::getInstance();
    
    // 초기 위치로 이동
    robot->movDefault();
    delay(1000); // 초기화 완료 대기
    
    robot->printCurrentPosition();
    Serial.println("[i] Setup complete. Ready for commands.");
    Serial.println("");
    Serial.println("Available commands:");
    Serial.println("  - ready?                    : Check if robot is ready");
    Serial.println("  - mov,default               : Move to default position");
    Serial.println("  - mov,arms,<m1>,<m2>,<m3>,<m4>,<m5> : Move all arm motors");
    Serial.println("  - mov,tongs,<degree>        : Move gripper");
    Serial.println("  - mov,motor,<index>,<degree> : Move individual motor (1-6)");
    Serial.println("  - status                    : Show current position");
    Serial.println("  - help                      : Show this help");
    Serial.println("");
}

void loop() 
{  
    if (Serial.available() < 1) {
        return;
    }

    String str = Serial.readStringUntil('\n');
    str.trim();
    
    if (str.length() == 0) {
        return;
    }

    Robot *robot = Robot::getInstance();

    if (str.equals("ready?")) 
    {
        Serial.println("true");
    }
    else if (str.equals("status"))
    {
        robot->printCurrentPosition();
    }
    else if (str.equals("help"))
    {
        Serial.println("Available commands:");
        Serial.println("  - ready?, status, help");
        Serial.println("  - mov,default");
        Serial.println("  - mov,arms,<m1>,<m2>,<m3>,<m4>,<m5>");
        Serial.println("  - mov,tongs,<degree>");
        Serial.println("  - mov,motor,<index>,<degree>");
    }
    else if (str.startsWith("mov,default"))
    {
        robot->movDefault();
        Serial.println("OK: Moved to default position.");
    }
    else if (str.startsWith("mov,arms")) 
    {
        // 예: mov,arms,90,90,90,90,90
        const int NUM_PARAMS = 5;
        int degrees[NUM_PARAMS];
        int last_idx = 9; // "mov,arms," 이후 시작

        bool parseError = false;
        for (int i = 0; i < NUM_PARAMS; i++) 
        {
            int comma_idx = str.indexOf(',', last_idx);
            String param;
            
            if (comma_idx == -1) 
            {
                if (i < NUM_PARAMS - 1) 
                {
                    Serial.println("ERR: Not enough parameters for mov,arms. Expected 5.");
                    parseError = true;
                    break;
                }
                param = str.substring(last_idx);
            } 
            else 
            {
                param = str.substring(last_idx, comma_idx);
            }
            
            degrees[i] = param.toInt();
            if (degrees[i] < DEGREE_MIN || degrees[i] > DEGREE_MAX) 
            {
                Serial.print("ERR: Parameter ");
                Serial.print(i + 1);
                Serial.println(" out of range (0-180).");
                parseError = true;
                break;
            }
            
            last_idx = comma_idx + 1;
        }

        if (!parseError) 
        {
            Serial.print("OK: Moving arms to (");
            for (int i = 0; i < NUM_PARAMS; i++) 
            {
                Serial.print(degrees[i]);
                if (i < NUM_PARAMS - 1) Serial.print(", ");
            }
            Serial.println(")");
            
            robot->movArms(degrees[0], degrees[1], degrees[2], degrees[3], degrees[4]);
        }
    }
    else if (str.startsWith("mov,tongs")) 
    {
        // 예: mov,tongs,100
        String param = str.substring(10);
        int toDegree = param.toInt();
        
        if (toDegree < DEGREE_MIN || toDegree > DEGREE_MAX) 
        {
            Serial.println("ERR: Tongs parameter out of range (0-180).");
        }
        else 
        {
            robot->movTongs(toDegree);
            Serial.print("OK: Moving tongs to ");
            Serial.println(toDegree);
        }
    }
    else if (str.startsWith("mov,motor"))
    {
        // 예: mov,motor,1,90
        int first_comma = str.indexOf(',', 4);
        int second_comma = str.indexOf(',', first_comma + 1);
        
        if (first_comma == -1 || second_comma == -1) {
            Serial.println("ERR: Invalid format. Use: mov,motor,<index>,<degree>");
            return;
        }
        
        int motorIndex = str.substring(first_comma + 1, second_comma).toInt();
        int degree = str.substring(second_comma + 1).toInt();
        
        robot->moveMotor(motorIndex, degree);
    }
    else 
    {
        Serial.println("ERR: Unknown command. Type 'help' for available commands.");
    }
}