#include <Wire.h>

// motor PWM 
const int R_MOTOR_PWM = 4;
const int L_MOTOR_PWM = 5; 

// change direction of motors
// A = 1, B = 1 -- brake to Vcc
// A = 1, B = 0 -- clockwise
// A = 0, B = 1 -- counterclockwise
// A = 0, B = 0 -- brake to GND
const int R_MOTOR_IN_A = 2;
const int R_MOTOR_IN_B = 3;
const int L_MOTOR_IN_A = 7;
const int L_MOTOR_IN_B = 6;

// Encoder setup
  // encoder Arduino ports
  const int L_ENCODER_A = 24;
  const int L_ENCODER_B = 25;
  const int R_ENCODER_A = 52;
  const int R_ENCODER_B = 53;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  
  // Arduino pins to motor driver
  pinMode(R_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_IN_A, OUTPUT);
  pinMode(R_MOTOR_IN_B, OUTPUT);
  pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(L_MOTOR_IN_A, OUTPUT);
  pinMode(L_MOTOR_IN_B, OUTPUT);

}

void loop() {
  
  // PWM values: 25% = 64; 50% = 127; 75% = 191; 100% = 255
  // control speed
  analogWrite(R_MOTOR_PWM, 255);
  analogWrite(L_MOTOR_PWM, 255);

  // turn counter-clockwise
  digitalWrite(R_MOTOR_IN_A, HIGH);
  digitalWrite(R_MOTOR_IN_B, LOW);
  digitalWrite(L_MOTOR_IN_A, LOW);
  digitalWrite(L_MOTOR_IN_B, HIGH);

  digitalRead(L_ENCODER_A);
  digitalRead(L_ENCODER_B);
  digitalRead(R_ENCODER_A);
  digitalRead(R_ENCODER_B);

  if (digitalRead(L_ENCODER_A)== HIGH) {
    Serial2.println("C");
  }

  else if (digitalRead(L_ENCODER_B) == LOW) {
    Serial2.println("B");
  }

//  Serial2.print(digitalRead(L_ENCODER_A));
//  Serial2.print("  ");
//  Serial2.print(digitalRead(L_ENCODER_B));
//  Serial2.print("  ");
//  Serial2.print(digitalRead(R_ENCODER_A));
//  Serial2.print("  ");
//  Serial2.println(digitalRead(R_ENCODER_B));

  delay(10);

}
