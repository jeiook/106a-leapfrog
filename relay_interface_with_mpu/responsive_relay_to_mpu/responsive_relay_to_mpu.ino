/**
 * 106A - Intro to Robotics
 * UC Berkeley FA2021
 * Authours:      Abanob Bostouros
 *                ------------
 *                ------------
 * prject:        Leap Frog
 * 
 * 
 * Iterface the Arduino uno with mpu6050 to get accelartion data, sample it for noise and printed to stream.
 */



/*
 * A basic circuit for controling motor in 2 rirections using 2 relays and a transistor
 * 
 * 
 * --------------------------------------------------- vcc
 *                       |
 *       ON/OFF pin3 ---|
 *                       |
 *                       |-------------- pin4   Low  normal right
 *                      /                       HIGH left
 *                 -----   ----
 *                 |          |
 *                 ---- M -----
 *                 |          |
 *                 ----    ---- 
 *                        /
 *                       |------------- pin4 
 *                       |
 * --------------------------------------------------- Grd                   
 */

// liberies needed to communicate with mpu6050
#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 gyro;
// accelartion in "g"
int16_t ax, ay, az;
// rotation in "degree/sec"
int16_t gx, gy, gz;

// Motor ON/OFF
#define MOTOR_POWER 3
// Flip motor direction from right to left
#define FLIP_MOTOR 4

#define OUTPUT_READABLE

void setup() {
  // join I2C bus 
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(38400);
    pinMode(MOTOR_POWER, OUTPUT);
    pinMode(FLIP_MOTOR, OUTPUT);
    Serial.println("Intialize Gyro connection");
    gyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(gyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop() {

  float angles[] = {0.0, 0.0, 0.0};
  read_tilt_expo_mov(angles, gyro, 500, 0.25);
  Serial.println("Tilt angles:");Serial.print("\t");
  Serial.print("x-tilt: ");Serial.println(angles[0]);Serial.print("\t");
  Serial.print("y-tilt: ");Serial.println(angles[1]);Serial.print("\t");
  Serial.print("z-tilt: ");Serial.println(angles[2]);
  
  if (angles[0] < -1) {
    digitalWrite(FLIP_MOTOR, HIGH);
    digitalWrite(MOTOR_POWER, HIGH);
    
    Serial.println("Stick pointing down");
    Serial.println("Motor runing to the left");
  }
  else if (angles[0] > 1){
    digitalWrite(MOTOR_POWER, HIGH);
    digitalWrite(FLIP_MOTOR, LOW);
    
    Serial.println("Stick pointing up");
    Serial.println("Motor runing to the right");
  }
  else {
    digitalWrite(MOTOR_POWER, LOW);
    Serial.println("Stick x is almost perfect");
     Serial.println("Motor off");
  }
  angles[0] = 0;
  angles[1] = 0;
  angles[2] = 0;
  delay(1);
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
