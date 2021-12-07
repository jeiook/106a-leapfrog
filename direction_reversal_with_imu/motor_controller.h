#include "definitions.h"

const float MOTOR_DEADZONE_ANGLE = 0.5;
long motor1_en_count = 0;
long motor2_en_count = 0;
float Kp = 1;
float Kd = 3;

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
  const bool FLIP_MOTOR;

  bool cw;

  long prevCount;
  long prevTime;

public:
  MotorController(int num, int ena, int enb, int pwm, int cw_ccw, int brake, bool flip) 
    : number(num), ENA(ena), ENB(enb), PWM(pwm), CW_CCW(cw_ccw), BRAKE(brake), FLIP_MOTOR(flip) {
      prevCount = 0;
      prevTime = 0;
      cw = true;
    }

  /*
   * Sets up connections and starting state of the motor, as well as
   * configuring the timer (which is necessary for effective brushless 
   * motor control).
   */
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

  /*
   * Moves the motor according to some defined control law
   */
  void move_motor(float max_angle, float angle, float ang_vel) {
    angle = abs(angle);
    ang_vel = abs(ang_vel);
    if (angle < MOTOR_DEADZONE_ANGLE) {
      digitalWrite(BRAKE, LOW);
      analogWrite(PWM, 255);
      return;
    }
    float control_term = Kp*angle + Kd*ang_vel;
    int val = map(control_term, Kp*MOTOR_DEADZONE_ANGLE, (Kp+Kd)*max_angle/2, 255, 0);
    digitalWrite(BRAKE, HIGH);
    analogWrite(PWM, val);
  }

  /*
   * Determines rotational direction of the motor depending on the 
   * angular displacement of the stick
   */
  void orient_motor(float angle) {
    if (FLIP_MOTOR) {
      angle = -angle;  
    }
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

  /*
   * Get the angular velocity of the reaction wheel, in units of rpm
   */
  float getVelocity() {
    float vel;
    if (number == 1) {
      vel = (float)(motor1_en_count - prevCount) / (millis() - prevTime) * 60000; 
      prevCount = motor1_en_count;
    }
    if (number == 2) {
      vel = (float)(motor2_en_count - prevCount) / (millis() - prevTime) * 60000;
      prevCount = motor2_en_count;
    }
    prevTime = millis();
  }
};
