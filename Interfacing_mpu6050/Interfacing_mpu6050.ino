/**
 * 106A - Intro to Robotics
 * UC Berkeley FA2021
 * Authour: Abanob Bostouros
 * prject: Self balancing stick
 * 
 * 
 * Iterface the Arduino uno with mpu6050 to get data, sample it for noise and printed to stream.
 */


// liberies needed to communicate with mpu6050
#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 gyro;
int16_t ax, ay, az;
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
  gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.println("Sample Data:"); Serial.print("\t");
  Serial.print("ax: "); Serial.println(ax); Serial.print("\t");
  Serial.print("ay: "); Serial.println(ay); Serial.print("\t");
  Serial.print("az: "); Serial.println(az); Serial.print("\t");
  Serial.print("gx: "); Serial.println(gx); Serial.print("\t");
  Serial.print("gy: "); Serial.println(gy); Serial.print("\t");
  Serial.print("gz: "); Serial.println(gz);
  delay(1000);
}
