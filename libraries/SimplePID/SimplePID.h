//
//  SimplePID.h
//  
//
//  Created by Andres Abrego on 7/21/15.
//
//

#ifndef SimplePID_h
#define SimplePID_h

#include "Arduino.h"

class SimplePID {
public:
    SimplePID(double Kp, double Ki, double Kd);
    void compute();
    void setConstants(double Kp, double Ki, double Kd);
    void input(double in);
    void setpoint(double set);
    double output();
private:
    unsigned long lastTime;
    double Input;
    double Setpoint;
    double Output;
    double errSum, lastErr;
    double kp, ki, kd;
};

#endif
