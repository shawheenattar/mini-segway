//
//  SimplePID.cpp
//  
//
//  Created by Andres Abrego on 7/21/15.
//
//

#include "Arduino.h"
#include "SimplePID.h"

SimplePID::SimplePID(double Kp, double Ki, double Kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void SimplePID::setConstants(double Kp, double Ki, double Kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void SimplePID::input(double in) {
    Input = in;
}

void SimplePID::setpoint(double set) {
    Setpoint = set;
}

double SimplePID::output() {
    return Output;
}

void SimplePID::compute() {
    /*How long since we last calculated*/
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime);
    
    /*Compute all the working error variables*/
    double error = Setpoint - Input;
    errSum += (error * timeChange);
    double dErr = (error - lastErr) / timeChange;
    
    /*Compute PID Output*/
    Output = kp * error + ki * errSum + kd * dErr;
    
    /*Remember some variables for next time*/
    lastErr = error;
    lastTime = now;
}