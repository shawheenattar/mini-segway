#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Simple_AHRS.h>

// Create sensor instances.
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

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

void setup() {
  Serial.begin(115200);
  
  // Arduino pins to motor driver
  pinMode(R_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_IN_A, OUTPUT);
  pinMode(R_MOTOR_IN_B, OUTPUT);
  pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(L_MOTOR_IN_A, OUTPUT);
  pinMode(L_MOTOR_IN_B, OUTPUT);

  Serial.println(F("Adafruit 9 DOF Board AHRS Example")); Serial.println("");
  
  // Initialize the sensors.
  accel.begin();
  mag.begin();
}

void loop() {
  // PWM values: 25% = 64; 50% = 127; 75% = 191; 100% = 255
  // control speed
  analogWrite(R_MOTOR_PWM, 127);
  analogWrite(L_MOTOR_PWM, 127);

  // turn counter-clockwise
  digitalWrite(R_MOTOR_IN_A, LOW);
  digitalWrite(R_MOTOR_IN_B, HIGH);
  digitalWrite(L_MOTOR_IN_A, LOW);
  digitalWrite(L_MOTOR_IN_B, HIGH);

  delay(10);

  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.heading);
    Serial.println(F(""));
  }
  
  delay(100);
}
