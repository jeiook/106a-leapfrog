/**
 * 106A - Intro to Robotics
 * UC Berkeley FA2021
 * Authours:      Abanob Bostouros, Jewook Ryu, Thanh Tran
 *                ------------
 *                ------------
 * project:        Leap Frog
 * 
 * 
 * Interface the Arduino uno with mpu6050 to get accelartion data, sample it for noise and use the signal to control motor
 */

#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <Servo.h>

#define MOTOR1_PWM_PORT 9
#define MOTOR1_BRAKE_PORT 8
#define MOTOR1_DIRECTION_PORT 3
#define MOTOR1_ENCODER_PORT_1 2
#define MOTOR1_ENCODER_PORT_2 5

Servo ESC;      // Servo object to control ESC

MPU6050 gyro;
// accelartion in "g"
int16_t ax, ay, az;
// rotation in "degree/sec"
int16_t gx, gy, gz;

long count = 0;
long prevCount = 0;
long prevTime = millis();

void setup() {
  // join I2C bus 
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(9600);
//    Serial.println("Intialize Gyro connection");
    gyro.initialize();
//    Serial.println("Testing device connections...");
//    Serial.println(gyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

//    Serial.println("Setting up motor");
    motor_setup();
    flip_ports_setup();
}

void loop() {
//  float angles[] = {0.0, 0.0, 0.0};
//  read_tilt_kalman(angles, gyro, 100);
//
////  Serial.println("Tilt angles:");Serial.print("\t");
////  Serial.print("x-tilt: ");Serial.println(angles[0]);Serial.print("\t");
////  Serial.print("y-tilt: ");Serial.println(angles[1]);Serial.print("\t");
////  Serial.print("z-tilt: ");Serial.println(angles[2]);
//
//  orient_motor(1, angles[0]);
//  move_motor(angles[0]);
//  
//  angles[0] = 0;
//  angles[1] = 0;
//  angles[2] = 0;

//  delay(5);

  for (int i = 0; i < 1000; i += 10) {
    ESC.write(i);
    Serial.print("motor output ");
    Serial.println(i);
    Serial.print((float)(count - prevCount) / (float) (millis() - prevTime) * 1000 * 60);
    Serial.println(" RPM");
    Serial.print("thing ");
    Serial.println(analogRead(A5));
    prevCount = count;
    prevTime = millis();
    delay(500);
  }
}

/*
  * Function to calculate tilt with Kalman filtered sample data
  * 
  * args :
  *       float*  angles    pointer to array to store tilt_angles value
  *       MPU6050 gyro      intialized object 
  *       int     samp_size number of iterution for doing avg sampling algorithm
  *       float   U         estimate
  */
void read_tilt_kalman(float* angles, MPU6050 gyro, int samp_size) {

  static int16_t U[3] = {angles[0], angles[1], angles[2]};
  static const double R = 40; /* noise covariance */
  static const double H = 1.00; /* measurement map scalar */
  static const double Q = 10; /* initial extimated covariance */
  static double P = 0; /* initial error covariance */
  static int16_t U_hat[3] = {0, 0, 0}; /* initial estimated state */
  static double K = 0; /* Kalman gain */
  
  gyro.getAcceleration(&U[0], &U[1], &U[2]);
  for (int i = 0; i < samp_size; ++i) {
      gyro.getAcceleration(&U[0], &U[1], &U[2]);
      
      K = P * H / (H * P * H + R); /* update kalman */
      U_hat[0] = U_hat[0] + K * (U[0] - H * U_hat[0]); /* update estimated */
      U_hat[1] = U_hat[1] + K * (U[1] - H * U_hat[1]);
      U_hat[2] = U_hat[2] + K * (U[2] - H * U_hat[2]);
      
      /* update covariance */
      P = (1 - K*H)*P + Q;

  }
  angles[0] = atan(U_hat[0] / (pow((pow(U_hat[1], 2) + pow(U_hat[2], 2)), 0.5))) * 180 / M_PI;
  angles[1] = atan(U_hat[1] / (pow((pow(U_hat[0], 2) + pow(U_hat[2], 2)), 0.5))) * 180 / M_PI;
  angles[2] = atan((pow((pow(U_hat[1], 2) + pow(U_hat[2], 2)), 0.5)) / U_hat[2]) * 180 / M_PI;
 }

void motor_setup() {
  ESC.attach(MOTOR1_PWM_PORT);

  pinMode(MOTOR1_ENCODER_PORT_1, INPUT);
  pinMode(MOTOR1_ENCODER_PORT_2, INPUT);
  digitalWrite(MOTOR1_ENCODER_PORT_1, LOW);
  digitalWrite(MOTOR1_ENCODER_PORT_2, LOW);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_PORT_1), readEncoder, CHANGE);
}

int move_motor(float angle) {
  angle = abs(angle);
  if (angle < 1) {
    digitalWrite(MOTOR1_BRAKE_PORT, HIGH);
    ESC.write(0);
    Serial.println("motor output 0");
  }
  int val = map(angle, 1, 90, 0, 180);
  Serial.print("motor output ");
  Serial.println(val);
  ESC.write(val);
}

void flip_ports_setup() {
  pinMode(MOTOR1_BRAKE_PORT, OUTPUT);
  digitalWrite(MOTOR1_BRAKE_PORT, HIGH);
  pinMode(MOTOR1_DIRECTION_PORT, OUTPUT);
  // todo: motor 2  
}

// number: 1 or 2
void orient_motor(int number, float angle) {
  if (number == 1) {
    if (angle >= 0) {
      digitalWrite(MOTOR1_DIRECTION_PORT, HIGH);
    } else {
      digitalWrite(MOTOR1_DIRECTION_PORT, LOW);
    } 
  }
}

void readEncoder() { 
  if(digitalRead(MOTOR1_ENCODER_PORT_1) == digitalRead(MOTOR1_ENCODER_PORT_2)) {
    --count;
  } else {
    ++count;
  }
}
