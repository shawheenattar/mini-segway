#include "Arduino.h"
#include <QueueList.h>
#include "PIDController_Claire.h"

PIDController_Claire::PIDController_Claire(float kP, float kI, float kD, bool integralQueue, bool pGainScheduling, bool derivativeKick)
{
  // gains
  _kP = kP;
  _kI = kI;
  _kD = kD;

  // custom P, I, D;
  _integralQueue = integralQueue;
  _pGainScheduling = pGainScheduling;
  _derivativeKick = derivativeKick;

  // private compute variables
  _error = 0;
  _errorSum = 0;
  _prevError = 0;
  _prevReading = 0;
}

float PIDController_Claire::compute(float currentReading, float setpoint)
{
  _error = currentReading - setpoint;
  return (pTerm(currentReading) + iTerm() + dTerm(currentReading));
}

float PIDController_Claire::feedforwardCompute(float currentReading, float setpoint, float kFF) {
  _feedback = compute(currentReading, setpoint);
  _feedforward = kFF * setpoint;
  return (_feedback + _feedforward);
}

float PIDController_Claire::pTerm(float currentReading)
{
  if (_pGainScheduling) {
    // special function for pitch to PWM
    //		_kP = min(abs(((55 * pow(currentReading, 2)) + (20 * currentReading) + 10)), kP_MAX);

    //                float scale = abs(currentReading)/2;
    //                _kP = min(scale * kP_MAX, kP_MAX);
    _kP = (abs(_error) > 10) ? kP_MAX : 0;
  }

  return (_kP * _error);
}

float PIDController_Claire::iTerm()
{
  if (_integralQueue) {
    // update queue and sum with current error
    _errorQueue.push(_error);
    _errorSum += _error;

    if (_errorQueue.count() > MAX_QUEUE_SIZE) { // keep most recent __ # of values
      _errorSum -= _errorQueue.pop();
    }

  } else {
    _errorSum += _error;
    // _printer->println(_errorSum);
  }

  return (_kI * _errorSum);
}

float PIDController_Claire::dTerm(float currentReading)
{
  
  if (_derivativeKick) {
    _dTerm = _kD * (currentReading - _prevReading);
    _prevReading = currentReading;
  } else {
    _dTerm = _kD * (_error - _prevError);
    _prevError = _error;
  }
  return _dTerm;
}

float PIDController_Claire::getErrorSum() {
  return _errorSum;
}

float PIDController_Claire::getDerivative() {
  return _dTerm;
}
