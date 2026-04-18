# PID System Documentation

## Where PID lives
- Core class: `include/PIDController.h`
- Tunables: `include/PIDConfig.h`
- Runtime use: `include/Chassis.h`

## Controller model
The controller output is:

$$u(t) = k_P e(t) + k_I \int e(t) dt + k_D \frac{de(t)}{dt}$$

Where:
- $e(t)$ is the current error.
- $k_P$ reacts to present error.
- $k_I$ reacts to accumulated past error.
- $k_D$ reacts to error rate of change.

## Anti-windup
Integral is clamped with `maxIntegral` to prevent runaway overshoot after long error periods.

## In this template
1. `drivePID`
- Controls linear distance traveled.

2. `turnPID`
- Controls heading angle error.

3. `sDriveDistancePID`
- Controls approach distance in simultaneous move+turn commands.

4. `sDriveHeadingPID`
- Controls heading correction while moving.

## Inline tuning
You can override configured gains per call.

Example:
```cpp
// kP, kI, kD override only for this command
// then normal config values remain available for other calls
d(Motion::Direction::Forward, 24, 90, 3000, {2.6, 0.0, 0.03});
```

## Practical tuning tips
1. Tune `kP` first until response is strong but not unstable.
2. Add a little `kD` to reduce oscillation.
3. Add small `kI` only if steady-state error remains.
4. Keep timeout realistic so failed commands terminate safely.
