#include <PID.h>
#include <Arduino.h>

PID::PID(double kp, double ki, double kd, double setpoint) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  goal = setpoint;
  before = millis();
}

void PID::setKs(double kp, double ki, double kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

void PID::setKp(double kp) {
  Kp = kp;
}

void PID::setKi(double ki) {
  Ki = ki;
}

void PID::setKd(double kd) {
  Kd = kd;
}

double PID::run(double input) {
  // time passed
  unsigned long now = millis();
  double time = (double)(now - before);

  // proportional = difference between input and goal
  double error = goal - input;
  // integral = sum of the error over time
  errorSum += (error * time);
  // derivative = difference of errors over time
  // improves stability
  double dError = (error - lastError) / time;

  //Proportional + Integral + Derivative
  double output = (Kp * error) + (Ki * errorSum) + (Kd * dError);

  //update the variables for the next iteration
  lastError = error;
  before = now;

  return output;
}
