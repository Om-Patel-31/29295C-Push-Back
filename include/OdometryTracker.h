#ifndef ODOMETRYTRACKER_H
#define ODOMETRYTRACKER_H

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// Simple 2D odometry for a differential-drive robot.
// Units and frame:
// - Linear units: inches. Heading: degrees.
// - Coordinate frame: +X is 0 deg, +Y is 90 deg, angles increase CCW.
// - `update()` expects delta wheel travel (inches) since last call and
//   an absolute IMU heading (deg).

class OdometryTracker {
public:
  // Global position (inches). x increases along 0 deg, y along 90 deg.
  double x = 0.0;
  double y = 0.0;

  // Heading in degrees (absolute IMU reading). 0 = +X, increases CCW.
  double heading = 0.0;

  // Physical params (inches):
  // - wheelBase: distance between left and right drive wheels (used if
  //   you later compute rotation from differential travel).
  // - trackingWheelRadius: encoder/tracking wheel radius (for conversions).
  double wheelBase = 12;
  double trackingWheelRadius = 3.25;

  // Advance pose by average wheel travel along IMU heading.
  // Parameters:
  // - leftDist/rightDist: incremental linear travel (inches) of drive
  //   wheels since last update. Positive = forward.
  // - imuHeading: absolute heading from IMU (degrees).
  // Behavior: uses avg distance = (left+right)/2 to step (x,y) along
  // the IMU heading and stores imuHeading in `heading`.
  void update(double leftDist, double rightDist, double imuHeading) {
    double avgDist = (leftDist + rightDist) / 2.0;

    // Use IMU heading (degrees) as the current orientation
    heading = imuHeading;

    // Convert heading to radians for trig functions
    double headingRad = heading * M_PI / 180.0;

    // Move the estimated position forward by avgDist along the heading
    x += avgDist * std::cos(headingRad);
    y += avgDist * std::sin(headingRad);
  }

  // Reset pose to (0,0,0) (inches, degrees)
  void reset() {
    x = 0.0;
    y = 0.0;
    heading = 0.0;
  }

  // Set pose explicitly (use to initialize), heading in degrees
  void setPosition(double newX, double newY, double newHeading) {
    x = newX;
    y = newY;
    heading = newHeading;
  }

  // Euclidean distance to target (inches)
  double distanceTo(double targetX, double targetY) {
    double dx = targetX - x;
    double dy = targetY - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  // Heading (deg) from current position toward target (absolute, same
  // 0=+X, CCW positive). Useful for aiming or path-following.
  double headingTo(double targetX, double targetY) {
    double dx = targetX - x;
    double dy = targetY - y;
    return std::atan2(dy, dx) * 180.0 / M_PI;
  }
};

#endif // ODOMETRYTRACKER_H
