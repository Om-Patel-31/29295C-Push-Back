# Odometry System Documentation

## Where odometry lives
- Pose math: `include/OdometryTracker.h`
- Sensor reads + fusion: `include/Chassis.h`

## Pose convention
- Position: inches.
- Heading: degrees.
- Coordinate frame: +X at 0 deg, +Y at 90 deg, CCW positive.

## Base update flow
1. Read left/right wheel travel from 4 rotation sensors.
2. Compute incremental left/right delta since last cycle.
3. Use inertial heading as global orientation.
4. Integrate average distance into $(x, y)$.

## Sensor fusion options
1. GPS fusion
- Blends odometry position and heading with GPS using alpha.
- Helps long-run drift but should be weighted lightly.

2. Distance fusion
- Blends forward displacement estimate with front distance-sensor change.
- Helps in approaches where field features are visible by sensor.

## Why this is reliable
- Rotation sensors provide direct wheel-travel tracking.
- IMU stabilizes angle reference.
- GPS and distance sensor provide correction channels when conditions are valid.

## Common pitfalls
1. Wrong rotation direction flags
- Robot appears to move backward in odometry.

2. Wrong inches-per-rev conversion
- Distance in code does not match tape measure.

3. Distance sensor sees no object
- Fusion automatically becomes less useful until a valid object is in range.
