#ifndef MOTIONAPI_H
#define MOTIONAPI_H

#include <cmath>

namespace Motion {

enum class Direction {
  Forward = 1,
  Reverse = -1
};

struct PIDTune {
  double kP = NAN;
  double kI = NAN;
  double kD = NAN;
};

struct SDriveTune {
  PIDTune distance;
  PIDTune heading;
  bool allowSwing = true;
};

inline constexpr double defaultSegmentDistanceIn = 12.0;

} // namespace Motion

#endif // MOTIONAPI_H
