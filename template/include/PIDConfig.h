#ifndef PIDCONFIG_H
#define PIDCONFIG_H

namespace PIDConfig {

struct Gains {
  double kP;
  double kI;
  double kD;
};

inline constexpr Gains drive = {2.0, 0.5, 0.01};
inline constexpr Gains turn = {0.5, 0.01, 0.05};
inline constexpr Gains sDriveDistance = {2.2, 0.0, 0.02};
inline constexpr Gains sDriveHeading = {1.2, 0.0, 0.06};

inline constexpr double driveToleranceIn = 1.0;
inline constexpr double turnToleranceDeg = 1.0;
inline constexpr double sDriveDistanceToleranceIn = 0.75;
inline constexpr double sDriveHeadingToleranceDeg = 1.5;

inline constexpr double maxIntegralDrive = 50.0;
inline constexpr double maxIntegralTurn = 30.0;
inline constexpr double maxIntegralSDriveDistance = 40.0;
inline constexpr double maxIntegralSDriveHeading = 25.0;

inline constexpr double sDriveBlendDistanceIn = 12.0;
inline constexpr double swingTurnThresholdDeg = 8.0;
inline constexpr double swingInsideScale = 0.20;
inline constexpr double gpsFusionAlpha = 0.15;

} // namespace PIDConfig

#endif // PIDCONFIG_H
