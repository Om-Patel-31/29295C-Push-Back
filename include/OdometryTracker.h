#ifndef ODOMETRYTRACKER_H
#define ODOMETRYTRACKER_H

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class OdometryTracker {
public:
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double wheelBase = 10;
  double trackingWheelRadius = 3.25;

  void update(double leftDist, double rightDist, double imuHeading) {
    double avgDist = (leftDist + rightDist) / 2.0;

    heading = imuHeading;

    double headingRad = heading * M_PI / 180.0;
    x += avgDist * std::cos(headingRad);
    y += avgDist * std::sin(headingRad);
  }

  void reset() {
    x = 0.0;
    y = 0.0;
    heading = 0.0;
  }

  void setPosition(double newX, double newY, double newHeading) {
    x = newX;
    y = newY;
    heading = newHeading;
  }

  double distanceTo(double targetX, double targetY) {
    double dx = targetX - x;
    double dy = targetY - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double headingTo(double targetX, double targetY) {
    double dx = targetX - x;
    double dy = targetY - y;
    return std::atan2(dy, dx) * 180.0 / M_PI;
  }
};

#endif // ODOMETRYTRACKER_H
