/**
 * 106A - Intro to Robotics
 * UC Berkeley FA2021
 * Author: Thanh T. Tran
 * Email: thanhtran@berkeley.edu
 * Project: LEAPFROG, Self-Balancing Stick
 * Source: https://www.youtube.com/watch?v=uOQk8SJso6Q
 * 
 * Interface DYS 1000KV Brushless DC motor (BLDC) with 30A electric speed controller (ESC)
 */

#include <Servo.h>
Servo ESC;      // Servo object to control ESC
int potValue;   // Value of potentiometer from analog pin

void setup() {
  ESC.attach(9);    // (pin, min PWM, max PWM) in microseconds
  ESC.write(0);
  delay(2000);
  ESC.write(1200);
  delay(2000);
  Serial.begin(38400);
}

// set min pwm = 0, max pwm = 1000 -> write vals: min = 70, max = 180

void loop() {
  // put your main code here, to run repeatedly:
//  potValue = analogRead(A0);
//  potValue = map(potValue, 0, 1023, 0, 180); // Map the potentiometer value [0, 1023] -> [0,180]
  for (int i = 0; i < 180; i += 2) {
    ESC.write(i);
    Serial.print(i);
    Serial.print("\n");
    delay(500);
  }
//  Serial.print(potValue);
//  Serial.print("\n");
}
