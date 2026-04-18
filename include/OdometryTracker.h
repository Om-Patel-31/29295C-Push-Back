#ifndef ODOMETRYTRACKER_H
#define ODOMETRYTRACKER_H

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// OdometryTracker
//
// Purpose:
// Maintain a global 2D pose estimate (x, y, heading) for differential drive.
//
// Coordinate frame used by this template:
// - Linear units: inches
// - Angular units: degrees
// - +X axis = 0 deg
// - +Y axis = 90 deg
// - Positive rotation is counter-clockwise (CCW)
//
// Data flow:
// 1) Chassis computes left/right incremental travel since last loop.
// 2) Chassis passes those deltas plus absolute IMU heading to update().
// 3) This tracker integrates position and stores latest heading.

class OdometryTracker {
public:
  // Global position in inches.
  double x = 0.0;
  double y = 0.0;

  // Heading in degrees from IMU reference. 0 = +X, increases CCW.
  double heading = 0.0;

  // Optional geometry values kept here for future model upgrades.
  double wheelBase = 12;
  double trackingWheelRadius = 3.25;

  // Advance pose by average wheel travel along IMU heading.
  // Parameters:
  // - leftDist/rightDist: incremental linear travel (inches) since last call.
  // - imuHeading: absolute IMU heading (deg).
  // Implementation notes:
  // - Uses average distance = (leftDist + rightDist) / 2.
  // - Uses imuHeading for global orientation.
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

  // Reset pose to origin.
  void reset() {
    x = 0.0;
    y = 0.0;
    heading = 0.0;
  }

  // Set pose explicitly (for autonomous initialization).
  void setPosition(double newX, double newY, double newHeading) {
    x = newX;
    y = newY;
    heading = newHeading;
  }

  // Euclidean distance to target point in inches.
  double distanceTo(double targetX, double targetY) {
    double dx = targetX - x;
    double dy = targetY - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  // Absolute heading (deg) from current pose to target point.
  double headingTo(double targetX, double targetY) {
    double dx = targetX - x;
    double dy = targetY - y;
    return std::atan2(dy, dx) * 180.0 / M_PI;
  }
};

#endif // ODOMETRYTRACKER_H
