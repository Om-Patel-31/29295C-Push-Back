/*
 * Copyright (c) 2026 Stratford Robotics 29295
 * Developer: Om Patel
 */

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cmath>

class PIDController {
public:
  double kP, kI, kD;
  double integral = 0.0;
  double prevError = 0.0;
  double maxIntegral = 50.0;
  double tolerance = 1.0;
  bool enabled = false;
  double lastP = 0.0;
  double lastI = 0.0;
  double lastD = 0.0;

  PIDController(double p, double i, double d) : kP(p), kI(i), kD(d) {}

  double calculate(double error, double dt) {
    lastP = kP * error;

    integral += error * dt;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;
    lastI = kI * integral;

    lastD = 0.0;
    if (dt > 0) {
      lastD = kD * (error - prevError) / dt;
    }

    prevError = error;

    return lastP + lastI + lastD;
  }

  void reset() {
    integral = 0.0;
    prevError = 0.0;
  }

  bool atTarget(double error) {
    return std::abs(error) < tolerance;
  }
};

#endif // PIDCONTROLLER_H
