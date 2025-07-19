#include <ESP32Servo.h>
#include <math.h>

#define MOTOR_PIN    (23)   // D6 pin

Servo servo;
int currDegree = 0;


int getMomentaryDelayByGauss(int fromDegree, int toDegree, int currDegree) {
  if (fromDegree == toDegree) {
    return 0;
  }
  if (fromDegree < toDegree) {
    currDegree = (currDegree > toDegree) ? toDegree : currDegree;
    currDegree = (currDegree < fromDegree) ? fromDegree : currDegree;
  }
  else {
    currDegree = (currDegree > fromDegree) ? fromDegree : currDegree;
    currDegree = (currDegree < toDegree) ? toDegree : currDegree;
  }

  float x = ((float)currDegree - fromDegree) / (toDegree - fromDegree);  // Min-Max Normalize

  const float MOMENTARY_DELAY_MS_MAX = 20.0;
  const float MOMENTARY_DELAY_MS_MIN = 5.0;
  const float MOMENTARY_DELAY_MS_DIFF = MOMENTARY_DELAY_MS_MAX - MOMENTARY_DELAY_MS_MIN;
  const float ROTATE_DEGREE_MAX = 180.0;
  const float B = 0.5;
  const float C = 0.235;
  float y = MOMENTARY_DELAY_MS_MAX - MOMENTARY_DELAY_MS_DIFF * exp(- ((x - B) * (x - B)) / (C * C)) * (abs(fromDegree-toDegree) / ROTATE_DEGREE_MAX);
  
  return round(y);
}


void setup() {
  servo.attach(MOTOR_PIN); 
}


void loop() {
   
  int fromDegree = currDegree;
  int toDegree = 90;
  for ( ; currDegree < toDegree; currDegree += 1) {
    servo.write(currDegree);
    int momentaryDelay = getMomentaryDelayByGauss(fromDegree, toDegree, currDegree);
    delay(momentaryDelay);
  }
  delay(2000);

  fromDegree = currDegree;
  toDegree = 0;
  for ( ; currDegree > toDegree; currDegree -= 1) {
    servo.write(currDegree);
    int momentaryDelay = getMomentaryDelayByGauss(fromDegree, toDegree, currDegree);
    delay(momentaryDelay);
  }
  delay(2000); 
  
}