// motor PWM 
const int R_MOTOR_PWM = 4;
const int L_MOTOR_PWM = 5; 

// change direction of motors
// A = 1, B = 1 -- brake to Vcc
// A = 1, B = 0 -- clockwise
// A = 0, B = 1 -- counterclockwise
// A = 0, B = 0 -- brake to GND
const int R_MOTOR_IN_A = 2;
const int R_MOTOR_IN_B = 4;
const int L_MOTOR_IN_A = 7;
const int L_MOTOR_IN_B = 6;

void setup() {
  Serial.begin(9600);
  
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
  analogWrite(R_MOTOR_PWM, 127);
  analogWrite(L_MOTOR_PWM, 127);

  // turn counter-clockwise
  digitalWrite(R_MOTOR_IN_A, HIGH);
  digitalWrite(R_MOTOR_IN_B, LOW);
  digitalWrite(L_MOTOR_IN_A, LOW);
  digitalWrite(L_MOTOR_IN_B, HIGH);

  delay(10);
}
