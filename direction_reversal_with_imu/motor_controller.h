#include "definitions.h"

const float MOTOR_DEADZONE_ANGLE = 0.25;
long motor1_en_count = 0;
long motor2_en_count = 0;

float motor1_vel = 0;
float motor2_vel = 0;

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

  long prevCount;
  long prevTime;

  float Kp;
  float Kd;
  float Ki;
  float Kv;

public:
  MotorController(int num, int ena, int enb, int pwm, int cw_ccw, int brake, bool flip, float p, float d, float i, float v) 
    : number(num), ENA(ena), ENB(enb), PWM(pwm), CW_CCW(cw_ccw), BRAKE(brake), FLIP_MOTOR(flip) {
      prevCount = 0;
      prevTime = 0;
      Kp = p;
      Kd = d;
      Ki = i;
      Kv = v;
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
    digitalWrite(BRAKE, HIGH);
    
    pinMode(CW_CCW, OUTPUT);
    digitalWrite(CW_CCW, HIGH);

    manual_move(250);
    delay(2000);
    getVelocity();

    manual_move(255);
  }

  /*
   * Manually provide analog output to motor
   */
  void manual_move(float val) {
    analogWrite(PWM, val);
  }

  /*
   * Moves the motor according to some defined control law
   */
  void move_motor(float max_angle, float angle, float ang_vel, float ang_accel, float ang_int) {
    float motor_vel;
    if (number == 1) {
      motor_vel = motor1_vel;
    } if (number == 2) {
      motor_vel = motor2_vel;
    }

//    float desired_motor_vel = (m*g*l*sin(angle) - Ic*ang_accel) * (millis() - ti) / If + motor_vel
    
    float p = Kp*angle;
    float d = Kd*ang_vel;
    float in = Ki*ang_int;
    float v = Kv*motor_vel;
    float control_term = p + d + in + v;
    orient_motor(control_term);
    control_term = abs(control_term);
    float max_term = abs((Kp+Kd+Ki)*max_angle) * 5;
    if (control_term > max_term) {
      control_term = max_term;
    }
    int val = 255 - map(control_term, 0, max_term, 2, 125);
    if (val < 125) {
      return;
    }
//    digitalWrite(BRAKE, HIGH);
    manual_move(val);
    
    Serial.print("motor ");
    Serial.print(number);
    Serial.print(": ");
    Serial.print("p=");
    Serial.print(p);
    Serial.print("; d=");
    Serial.print(d);
    Serial.print("; in=");
    Serial.print(in);
    Serial.print("; v=");
    Serial.print(v);
    Serial.print(" u=");
    Serial.print(control_term);
    Serial.print(" out of ");
    Serial.print(max_term);
    Serial.print(" ---> ");
    Serial.println(val);
  }

  /*
   * Determines rotational direction of the motor depending on the control term
   */
  void orient_motor(float control_term) {
    if (FLIP_MOTOR) {
      control_term = -control_term;
    }
    if (control_term >= 0) {
      digitalWrite(CW_CCW, HIGH);
    } else {
      digitalWrite(CW_CCW, LOW);
    }
  }

  /*
   * Get the angular velocity of the reaction wheel in rotations per second
   */
  float getVelocity() {
    float vel;
    if (number == 1) {
      motor1_vel = (float)(motor1_en_count - prevCount) / (millis() - prevTime) * 1000;
      vel = motor1_vel;
      prevCount = motor1_en_count;
    }
    if (number == 2) {
      motor2_vel = (float)(motor2_en_count - prevCount) / (millis() - prevTime) * 1000;
      vel = motor2_vel;
      prevCount = motor2_en_count;
    }
    prevTime = millis();
    return vel;
  }

  /*
   * Prints the encoder values
   *  motor_en_count := cumulative count
   *  prevCount := most recent count when calculating angular velocity
   *  prevTime := most recent time when calculating angular velocity
   */
  void print_encoders() {
    if (number == 1) {
      Serial.println("motor 1");
      Serial.print(motor1_en_count);
      Serial.print("; ");
      Serial.print(prevCount);
      Serial.print("; ");
      Serial.println((millis() - prevTime));
      Serial.println("----------------------------------");
    } else if (number == 2) {
      Serial.println("motor 2");
      Serial.print(motor2_en_count);
      Serial.print("; ");
      Serial.print(prevCount);
      Serial.print("; ");
      Serial.println((millis() - prevTime));
      Serial.println("----------------------------------");
    }
  }


/********************* Nidec 24H Motor Characteristics ***********************/

  /*
   * Sweeps through PWM values and prints the velocity at each PWM.
   * The purpose of this test is to determine the linearity of our motor velocity with respect
   *  to time and PWM.
   * To conduct this test, change the delay value and examine how the Serial Plotter changes.
   */
  void sweep_pwm() {
    int time_delay = 5;
    
    Serial.println("Sweeping up...");
    for (float val = 255; val > 0; val -= 1) {
      manual_move(val);
      Serial.print(val);
      Serial.print(" -------> ");
      Serial.println(getVelocity());
      getVelocity();
      delay(time_delay);
    }
    Serial.println("Sweeping down...");
    delay(1000);
    for (float val = 0; val < 255; val += 1) {
      manual_move(val);
      Serial.print(val);
      Serial.print(" -------> ");
      Serial.println(getVelocity());
      getVelocity();
      delay(time_delay);
    }
  }

  /*
   * Sets the PWM values from LOW -> HIGH very quickly and prints the velocity at each PWM.
   * The purpose of this test is to characterize the time delay between our set PWM and output
   *  velocity.
   * To conduct this test, change the delay value and examine how the Serial Plotter changes.
   */
  void oscillate_pwm() {
    int time_delay = 250;

    manual_move(255);
    Serial.print("255");
    Serial.print(" -------> ");
    Serial.println(getVelocity());
    delay(time_delay);
    manual_move(0);
    Serial.print("0");
    Serial.print(" -------> ");
    Serial.println(getVelocity());
    delay(time_delay);
  }
};
