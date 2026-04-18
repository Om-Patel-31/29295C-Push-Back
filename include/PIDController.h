#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cmath>

// PIDController
// Professional but lightweight PID utility used by drive, turn, and s_drive.
//
// Control law:
// output = kP*error + kI*integral(error) + kD*derivative(error)
//
// Design goals for this template:
// 1) Easy for beginners to tune.
// 2) Safe by default (integral clamping).
// 3) Introspectable (lastP/lastI/lastD are exposed for debugging).
//
// Units are intentionally generic:
// - If error is inches, output might be motor percent.
// - If error is degrees, output might be turn command percent.
// Keep units consistent in calling code.

class PIDController {
public:
  // PID gains.
  double kP, kI, kD;

  // Internal state for integral and derivative calculation.
  double integral = 0.0;
  double prevError = 0.0;

  // Protection and convergence helpers.
  double maxIntegral = 50.0; // clamp to avoid integral wind-up
  double tolerance = 1.0;    // used by atTarget
  bool enabled = false;

  // Exposed last computed terms for debugging/tuning.
  double lastP = 0.0;
  double lastI = 0.0;
  double lastD = 0.0;

  PIDController(double p, double i, double d) : kP(p), kI(i), kD(d) {}

  // calculate(error, dt)
  // - error: current process error in your chosen units.
  // - dt: loop timestep in seconds.
  // Returns: summed PID output.
  // Notes:
  // - derivative term is skipped if dt <= 0.
  // - integral is clamped to +/- maxIntegral.
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

  // Reset integrator and derivative history between commands.
  void reset() {
    integral = 0.0;
    prevError = 0.0;
  }

  // Quick convergence check against tolerance.
  bool atTarget(double error) {
    return std::abs(error) < tolerance;
  }
};

#endif // PIDCONTROLLER_H
