#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
  public:
    Motor(int pwmPort, int aPort, int bPort);
    void sendPWMCommand(int command);
    void setup();

  private:
    void moveMotor(int speed, int direction);
    int _pwmPort, _aPort, _bPort;
    int _speed, _direction;

    // directions
    static const int FORWARD = 0;
    static const int BACKWARD = 1;
};

#endif
