#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cmath>

// Lightweight PID controller helper.
// - `kP`, `kI`, `kD` are the proportional, integral, and derivative gains.
// - Inputs/outputs are raw; scale to match motor/actuator ranges externally.

class PIDController {
public:
  // PID gains
  double kP, kI, kD;

  // Internal state for integral and derivative calculation
  double integral = 0.0;
  double prevError = 0.0;

  // Protection and tuning helpers
  double maxIntegral = 50.0; // clamp to avoid integral wind-up
  double tolerance = 1.0;    // used by `atTarget`
  bool enabled = false;

  // Exposed last computed terms for debugging/tuning
  double lastP = 0.0;
  double lastI = 0.0;
  double lastD = 0.0;

  PIDController(double p, double i, double d) : kP(p), kI(i), kD(d) {}

  // calculate(error, dt): compute PID output for given error and
  // timestep `dt` (seconds). dt==0 will skip the derivative term.
  // Returns the sum P+I+D. Integral is clamped by `maxIntegral`.
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

  // Reset integrator and derivative history
  void reset() {
    integral = 0.0;
    prevError = 0.0;
  }

  // Quick check if error is within tolerance
  bool atTarget(double error) {
    return std::abs(error) < tolerance;
  }
};

#endif // PIDCONTROLLER_H
