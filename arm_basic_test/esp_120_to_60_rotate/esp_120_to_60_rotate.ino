#include <ESP32Servo.h>

/* ===== 서보 PWM 핀 매핑 ===== */
#define PIN_BASE        23   // J1
#define PIN_SHOULDER    22   // J2
#define PIN_ELBOW       21   // J3
#define PIN_WRIST_PITCH 19   // J4
#define PIN_WRIST_ROLL  18   // J5
#define PIN_GRIPPER     17   // J6

class Motor6DOF {
private:
  Servo s[6];  // 0:BASE 1:SHOULDER 2:ELBOW 3:WRIST_PITCH 4:WRIST_ROLL 5:GRIPPER

public:
  void attachAll() {
    s[0].attach(PIN_BASE,        1000, 2000);
    s[1].attach(PIN_SHOULDER,    1000, 2000);
    s[2].attach(PIN_ELBOW,       1000, 2000);
    s[3].attach(PIN_WRIST_PITCH, 1000, 2000);
    s[4].attach(PIN_WRIST_ROLL,  1000, 2000);
    s[5].attach(PIN_GRIPPER,     1000, 2000);
  }

  void defaultPos() {
    for (int i = 0; i < 6; ++i)
      s[i].write(i == 5 ? 60 : 90);   // 그리퍼만 60°, 나머지 90°
  }

  /* 단일 축 테스트용 스윕 */
  void sweep(int idx) {
    if (idx < 0 || idx > 5) return;
    s[idx].write(120); delay(800);
    s[idx].write( 60); delay(800);
    s[idx].write(idx == 5 ? 60 : 90); delay(600);
  }

  /* 원래 예제와 동일한 시퀀스 */
  void demoSequence() {
    for (int i = 0; i < 6; ++i) sweep(i);
  }
};

/* ===== 전역 인스턴스 ===== */
Motor6DOF arm;

void setup() {
  /* 서보용 하드웨어 타이머 0~3 모두 예약 */
  for (int t = 0; t < 4; ++t) ESP32PWM::allocateTimer(t);

  arm.attachAll();
  arm.defaultPos();
}

void loop() {
  arm.demoSequence();      // J1→J6 차례로 120°↔60° 스윕
  delay(1000);
}
