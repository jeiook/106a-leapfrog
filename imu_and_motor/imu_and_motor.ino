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

#define MOTOR_SIGNAL_PORT 9

Servo ESC;      // Servo object to control ESC

MPU6050 gyro;
// accelartion in "g"
int16_t ax, ay, az;
// rotation in "degree/sec"
int16_t gx, gy, gz;

void setup() {
  // join I2C bus 
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(9600);
    Serial.println("Intialize Gyro connection");
    gyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(gyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Serial.println("Setting up motor");
    motor_setup();
}

void loop() {
  float angles[] = {0.0, 0.0, 0.0};
//  read_tilt_expo_mov(angles, gyro, 500, 0.25);
//  read_tilt_kalman(angles, gyro, 50);
  read_tilt_kalman(angles, gyro, 500);
  Serial.println("Tilt angles:");Serial.print("\t");
  Serial.print("x-tilt: ");Serial.println(angles[0]);Serial.print("\t");
  Serial.print("y-tilt: ");Serial.println(angles[1]);Serial.print("\t");
  Serial.print("z-tilt: ");Serial.println(angles[2]);

  move_motor(angles[0]);
  
  angles[0] = 0;
  angles[1] = 0;
  angles[2] = 0;
  delay(50);
}
/*
  * Function to calculate tilt with exponitial moving ave sample data
  * 
  * args :
  *       float*  angles    pointer to array to store tilt_angles value
  *       MPU6050 gyro      intialized object 
  *       int     samp_size number of iterution for doing avg sampling algorithm
  *       float   alpha     weight of the most recent value
  */
void read_tilt_expo_mov(float* angles, MPU6050 gyro, int samp_size, float alpha) {
  int16_t cur_x, cur_y, cur_z;
  int16_t x, y, z;
  gyro.getAcceleration(&x, &y, &z);
  for (int i = 0; i < samp_size; ++i) {
    gyro.getAcceleration(&cur_x, &cur_y, &cur_z);
    cur_x = (alpha * x) + ((1-alpha) * cur_x);
    cur_y = (alpha * y) + ((1-alpha) * cur_y);
    cur_z = (alpha * z) + ((1-alpha) * cur_z);
    x = cur_x; y = cur_y; z = cur_z;
  }
  angles[0] = atan(cur_x / (pow((pow(cur_y, 2) + pow(cur_z, 2)), 0.5))) * 180 / M_PI;
  angles[1] = atan(cur_y / (pow((pow(cur_x, 2) + pow(cur_z, 2)), 0.5))) * 180 / M_PI;
  angles[2] = atan((pow((pow(cur_y, 2) + pow(cur_z, 2)), 0.5)) / cur_z) * 180 / M_PI;
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
  ESC.attach(MOTOR_SIGNAL_PORT);    // (pin, min PWM, max PWM) in microseconds
  ESC.write(0);
  delay(2000);
  ESC.write(1000);
  delay(2000);
  Serial.begin(9600);
  Serial.println("starting in 3 sec");
  for (int i = 30; i < 50; i += 4) {
    ESC.write(i);
    delay(500);
  }
  delay(500);
}

int move_motor(float angle) {
  angle = abs(angle);
  if (angle < 2.5) {
    ESC.write(30);
    Serial.println("motor output 30");
  }
  int val = map(angle, 2.5, 90, 56, 180);
  Serial.print("motor output ");
  Serial.println(val);
  ESC.write(val);
}
