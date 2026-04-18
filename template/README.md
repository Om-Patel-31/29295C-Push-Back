# VEX Drive Template (4-Encoder Odom + GPS + s_drive)

This folder is a quick-start template for future projects.

## What to copy
- `template/include/PIDConfig.h` into your project `include/` folder.
- `template/src/auton_template.cpp` examples into your main auton file.

## Tuning workflow
1. Tune `PIDConfig::turn` with in-place turns first.
2. Tune `PIDConfig::drive` for straight line distance.
3. Tune `PIDConfig::sDriveHeading` so heading settles smoothly.
4. Tune `PIDConfig::sDriveDistance` so point approach is stable.
5. Adjust `gpsFusionAlpha` from 0.05 to 0.25 based on field GPS noise.

## Notes
- `rotationInchesPerRev` in Chassis must match your wheel setup.
- For highest accuracy, reset odometry and sensor positions before auton.
- `s_drive` supports swing behavior for faster heading capture.
