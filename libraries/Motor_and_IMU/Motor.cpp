#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int pwmPort, int aPort, int bPort)
{
  _pwmPort = pwmPort;
  _aPort = aPort;
  _bPort = bPort;
}

void Motor::setup()
{
  pinMode(_pwmPort, OUTPUT);
  pinMode(_aPort, OUTPUT);
  pinMode(_bPort, OUTPUT);
}

void Motor::sendPWMCommand(int command)
{
  _speed = min(abs(command), 255);
  _direction = (command >= 0) ? FORWARD : BACKWARD;
  moveMotor(_speed, _direction);
}

void Motor::moveMotor(int speed, int direction) {
  analogWrite(_pwmPort, speed);
  digitalWrite(_aPort, direction);
  digitalWrite(_bPort, abs(1 - direction));
}
