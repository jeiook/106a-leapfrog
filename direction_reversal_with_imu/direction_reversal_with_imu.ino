/**
 * 106A - Intro to Robotics
 * UC Berkeley FA2021
 * Authours:      Abanob Bostouros, Katja Maucec, Jewook Ryu, Thanh Tran
 *                ------------
 *                ------------
 * project:        106a LeapFrog
 * 
 * 
 * Use body rotational data from MPU 6050 IMU to control 
 * the reaction wheel motors which balance the body
 */

#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <Servo.h>
#include "definitions.h"
#include "motor_controller.h"
#include "imu.h"

IMU imu;
MotorController mc1 = MotorController(1, MOTOR1_ENA, MOTOR1_ENB, MOTOR1_PWM, MOTOR1_CW_CCW, MOTOR1_BRAKE);
MotorController mc2 = MotorController(2, MOTOR2_ENA, MOTOR2_ENB, MOTOR2_PWM, MOTOR2_CW_CCW, MOTOR2_BRAKE);

void setup() {
  Serial.begin(9600);
  imu.setup();
  mc1.setup();
  mc2.setup();
}

void loop() {
  mc1.orient_motor(imu.get_angle(0));
  mc2.orient_motor(-imu.get_angle(1));
  mc1.move_motor(imu.get_angle(0));
  mc2.move_motor(imu.get_angle(1));
  delay(5);
}
