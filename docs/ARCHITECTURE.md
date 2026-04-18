# Professional Motion Template Architecture

This project is organized so beginners can call very short functions, while advanced users can tune full closed-loop behavior.

## Layers
1. Public motion API
- Main call surface for autonomous: `d`, `turn`, `s_drive`, and shorthand aliases.
- Goal: easy calls first, optional tuning second.

2. Chassis control layer
- File: `include/Chassis.h`
- Owns sensor fusion, PID loops, drivetrain outputs, and motion command execution.

3. Core control math
- File: `include/PIDController.h`
- Generic PID logic with anti-windup and diagnostics.
- File: `include/OdometryTracker.h`
- Pose tracking in inches/degrees.

4. Tunables
- File: `include/PIDConfig.h`
- Single source of tuning values for drive/turn/s_drive and fusion behavior.

## Sensor stack
1. Rotation sensors (4 total)
- Primary wheel-travel source for odometry and distance control.

2. Inertial sensor
- Primary heading source.

3. GPS
- Optional position/heading fusion to reduce drift over long paths.

4. Distance sensor
- Optional forward-distance fusion to improve close-range approach consistency.

## Student usage style
1. Fast calls
- `d(Direction::Forward)`
- `d(Direction::Reverse, 20)`

2. Tuned calls
- `d(Direction::Forward, 30, 100, 4000, {2.8, 0.0, 0.04})`

3. Point + heading calls
- `sd(36, 24, 90)`

## Tuning sequence
1. Turn PID
2. Drive PID
3. s_drive heading PID
4. s_drive distance PID
5. GPS and distance fusion alpha
