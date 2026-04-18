#ifndef PIDCONFIG_H
#define PIDCONFIG_H

// Keep all tuneable values here so tuning sessions only touch one file.
namespace PIDConfig {

struct Gains {
  double kP;
  double kI;
  double kD;
};

inline constexpr Gains drive = {2.0, 0.5, 0.01};
inline constexpr Gains turn = {0.5, 0.01, 0.05};

// s_drive uses distance and heading controllers together.
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

// Heading blend for s_drive:
// Far away -> point toward travel direction.
// Near target -> lock to final heading.
inline constexpr double sDriveBlendDistanceIn = 12.0;

// Swing-turn behavior for s_drive.
inline constexpr double swingTurnThresholdDeg = 8.0;
inline constexpr double swingInsideScale = 0.20;

// GPS fusion alpha: 0 = odo only, 1 = GPS only.
inline constexpr double gpsFusionAlpha = 0.15;

// Distance sensor fusion for forward/back distance PID.
// 0 = ignore distance sensor, 1 = trust distance sensor fully.
inline constexpr double distanceFusionAlpha = 0.20;

// Ignore distance readings above this range (sensor too far/no target).
inline constexpr double distanceMaxValidIn = 80.0;

// Minimum delta before distance sensor is considered informative.
inline constexpr double distanceMinDeltaIn = 0.25;

} // namespace PIDConfig

#endif // PIDCONFIG_H
