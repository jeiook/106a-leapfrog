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
MotorController mc1 = MotorController(1, MOTOR1_ENA, MOTOR1_ENB, MOTOR1_PWM, MOTOR1_CW_CCW, MOTOR1_BRAKE, true, 0.4, 0.8, 0.01, -0.05);
MotorController mc2 = MotorController(2, MOTOR2_ENA, MOTOR2_ENB, MOTOR2_PWM, MOTOR2_CW_CCW, MOTOR2_BRAKE, true, 0.4, 0.8, 0.01, -0.05);

void setup() {
  Serial.begin(9600);
  imu.setup();
  mc1.setup();
  mc2.setup();
  Serial.print("\n\ndone calibrating motors!\n\n");
}

void loop() {
//  mc1.getVelocity();
//  delay(100);
//  mc1.oscillate_pwm();

//  imu.read_tilt_expo_mov(100, 0.5);
  imu.read_tilt_kalman(100);
  mc1.move_motor(imu.max_angle(), imu.get_angle(2), imu.get_angular_vel(2), imu.get_angular_acc(2), imu.get_angular_int(2));
  mc2.move_motor(imu.max_angle(), imu.get_angle(1), imu.get_angular_vel(1), imu.get_angular_acc(1), imu.get_angular_int(1));
}
