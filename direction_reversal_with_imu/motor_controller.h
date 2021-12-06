#include "definitions.h"

long motor1_en_count = 0;
long motor2_en_count = 0;

void readEncoder1() {
  if(digitalRead(MOTOR1_ENA) == digitalRead(MOTOR1_ENB)) {
    --motor1_en_count;
  } else {
    ++motor1_en_count;
  }
}

void readEncoder2() {
  if(digitalRead(MOTOR2_ENA) == digitalRead(MOTOR2_ENB)) {
    --motor2_en_count;
  } else {
    ++motor2_en_count;
  }
}

class MotorController {
  const int number;
  const int ENA;
  const int ENB;
  const int PWM;
  const int CW_CCW;
  const int BRAKE;

  bool cw;

  long prevCount;
  long prevTime;

public:
  /*
   * Sets up arduino-facing motor ports
   */
  MotorController(int num, int ena, int enb, int pwm, int cw_ccw, int brake) 
    : number(num), ENA(ena), ENB(enb), PWM(pwm), CW_CCW(cw_ccw), BRAKE(brake) {
      prevCount = 0;
      prevTime = 0;
      cw = true;
    }

  void setup() {
    pinMode(PWM, OUTPUT);

    TCCR1B = TCCR1B & 0b11111000 | 0x04;

    pinMode(ENA, INPUT);
    pinMode(ENB, INPUT);
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
    if (number == 1) {
      attachInterrupt(digitalPinToInterrupt(ENA), readEncoder1, CHANGE); 
    }
    if (number == 2) {
      attachInterrupt(digitalPinToInterrupt(ENA), readEncoder2, CHANGE);
    }

    pinMode(BRAKE, OUTPUT);
    digitalWrite(BRAKE, LOW);
    
    pinMode(CW_CCW, OUTPUT);
    digitalWrite(CW_CCW, HIGH);
    prevTime = millis();
  }

  void move_motor(float angle) {
    angle = abs(angle);
    if (angle < 0.5) {
      digitalWrite(BRAKE, LOW);
      analogWrite(PWM, 255);
      return;
    }
    digitalWrite(BRAKE, HIGH);
    int val = map(angle, 1, 90, 255, 0);
    analogWrite(PWM, val);
  }

  void orient_motor(float angle) {
    if (angle >= 0) {
      if (!cw) {
        digitalWrite(BRAKE, LOW);  
      }
      digitalWrite(CW_CCW, HIGH);
      cw = true;
    } else {
      if (cw) {
        digitalWrite(BRAKE, LOW);  
      }
      digitalWrite(CW_CCW, LOW);
      cw = false;
    }
    digitalWrite(BRAKE, HIGH);
  }

  float getVelocity() {
    float vel;
    if (number == 1) {
      vel = (float)(motor1_en_count - prevCount) / (millis() - prevTime); 
      prevCount = motor1_en_count;
    }
    if (number == 2) {
      vel = (float)(motor2_en_count - prevCount) / (millis() - prevTime);
      prevCount = motor2_en_count;
    }
    prevTime = millis();
  }
};
