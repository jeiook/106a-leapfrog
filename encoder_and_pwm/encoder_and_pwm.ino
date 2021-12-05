#include <Servo.h>

#define MOTOR_SIGNAL 9
#define ENCODER_A 2
#define ENCODER_B 5
#define BRAKE 7
#define DIR_CTRL 8

Servo motor;

void setup() {
  // put your setup code here, to run once:
  pinMode(MOTOR_SIGNAL, OUTPUT);
  TCCR1B = TCCR1B & 0b11111000 | 0x04;
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIR_CTRL, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  digitalWrite(ENCODER_A, LOW);
  digitalWrite(ENCODER_B, LOW);
  digitalWrite(DIR_CTRL, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, CHANGE);
  Serial.begin(9600);
}

long count = 0;
long prevCount = 0;
long prevTime = millis();

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(MOTOR_SIGNAL, 0);
  Serial.println((count - prevCount) / (millis() - prevTime));
  prevTime = millis();
  prevCount = count;

  delay(1);
}

void readEncoder() {
  if(digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    --count;
  } else {
    ++count;
  }
}
