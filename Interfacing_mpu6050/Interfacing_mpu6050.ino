/**
 * 106A - Intro to Robotics
 * UC Berkeley FA2021
 * Authour: Abanob Bostouros
 * prject: leap Frog
 * 
 * 
 * Iterface the Arduino uno with mpu6050 to get accelartion data, sample it for noise and printed to stream.
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


#define OUTPUT_READABLE

void setup() {
  // join I2C bus 
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(38400);

    Serial.println("Intialize Gyro connection");
    gyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(gyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop() {
//  gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//      Serial.println("Sample Data:"); Serial.print("\t");
//    Serial.print("ax: "); Serial.println(ax); Serial.print("\t");
//    Serial.print("ay: "); Serial.println(ay); Serial.print("\t");
//    Serial.print("az: "); Serial.println(az); Serial.print("\t");
//    Serial.print("gx: "); Serial.println(gx); Serial.print("\t");
//    Serial.print("gy: "); Serial.println(gy); Serial.print("\t");
//    Serial.print("gz: "); Serial.println(gz);
  float angels[] = {0.0, 0.0, 0.0};
  read_tilt_expo_mov(angels, gyro, 500, 0.25);
  Serial.println("Tilt angles:");Serial.print("\t");
  Serial.print("x-tilt: ");Serial.println(angels[0]);Serial.print("\t");
  Serial.print("y-tilt: ");Serial.println(angels[1]);Serial.print("\t");
  Serial.print("z-tilt: ");Serial.println(angels[2]);
  angels[0] = 0;
  angels[1] = 0;
  angels[2] = 0;
  delay(1000);
}


 /*
  * Function to calculate tilt with ave sample data
  * 
  * args :
  *       float*  angles    pointer to array to store tilt_angles value
  *       MPU6050 gyro      intialized object 
  *       int     samp_size number of iterution for doing avg sampling algorithm
  */
void read_tilt_ave(float* angels, MPU6050 gyro, int samp_size) {
  // accelartion in "g"
  int16_t x, y, z;
  int16_t lax = 0;
  int16_t lay = 0;
  int16_t laz = 0;
  for(int i = 0; i < samp_size; ++i) {
    gyro.getAcceleration(&x, &y, &z);
    lax += x;
    lay += y;
    laz += z;
  }
  lax /= samp_size;lay /= samp_size;laz /= samp_size;
  angels[0] = atan(lax / (pow((pow(lay, 2) + pow(laz, 2)), 0.5))) * 180 / M_PI;
  angels[1] = atan(lay / (pow((pow(lax, 2) + pow(laz, 2)), 0.5))) * 180 / M_PI;
  angels[2] = atan((pow((pow(lay, 2) + pow(laz, 2)), 0.5)) / laz) * 180 / M_PI;
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
void read_tilt_expo_mov(float* angels, MPU6050 gyro, int samp_size, float alpha) {
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
  angels[0] = atan(cur_x / (pow((pow(cur_y, 2) + pow(cur_z, 2)), 0.5))) * 180 / M_PI;
  angels[1] = atan(cur_y / (pow((pow(cur_x, 2) + pow(cur_z, 2)), 0.5))) * 180 / M_PI;
  angels[2] = atan((pow((pow(cur_y, 2) + pow(cur_z, 2)), 0.5)) / cur_z) * 180 / M_PI;
 }