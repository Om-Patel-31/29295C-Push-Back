#ifndef CHASSIS_H
#define CHASSIS_H

#include "PIDController.h"
#include "OdometryTracker.h"
#include "vex.h"
#include <cmath>

class Chassis {
public:
    vex::motor_group& left;
    vex::motor_group& right;
    vex::inertial& imu;
    OdometryTracker odom;
    PIDController drivePID;
    PIDController turnPID;
    PIDController curvePID;

    Chassis(vex::motor_group& l, vex::motor_group& r, vex::inertial& i)
        : left(l), right(r), imu(i),
          drivePID(3.5, 0.10, 1.0),
          turnPID(0.5, 0.1, 0.05),
          curvePID(0.75, 0.5, 5) {}

    void reset() {
        left.resetPosition();
        right.resetPosition();
        odom.reset();
        drivePID.reset();
        turnPID.reset();
        curvePID.reset();
    }

    void updateOdometry() {
        double heading = imu.rotation();
        double leftPos = left.position(vex::rotationUnits::rev) * 10.2;
        double rightPos = right.position(vex::rotationUnits::rev) * 10.2;
        static double prevLeft = 0.0, prevRight = 0.0;
        double leftDelta = leftPos - prevLeft;
        double rightDelta = rightPos - prevRight;
        odom.update(leftDelta, rightDelta, heading);
        prevLeft = leftPos;
        prevRight = rightPos;
    }

    // Synchronous straight drive with PID, with optional inline tuning
    void drive(double dist, double maxSpeed = 100, int timeout = 5000,
               double kP = NAN, double kI = NAN, double kD = NAN) {
        const double dt = 0.01;
        int elapsed = 0;
        double startX = odom.x, startY = odom.y, startH = odom.heading;
        if (!std::isnan(kP)) drivePID.kP = kP;
        if (!std::isnan(kI)) drivePID.kI = kI;
        if (!std::isnan(kD)) drivePID.kD = kD;
        drivePID.tolerance = 1.0;
        drivePID.reset();
        while (elapsed < timeout) {
            updateOdometry();
            double dx = odom.x - startX;
            double dy = odom.y - startY;
            double headingRad = startH * M_PI / 180.0;
            double curDist = dx * std::cos(headingRad) + dy * std::sin(headingRad);
            double error = dist - curDist;
            if (drivePID.atTarget(error)) break;
            double out = drivePID.calculate(error, dt);
            out = std::fmax(std::fmin(out, maxSpeed), -maxSpeed);
            left.setVelocity(std::abs(out), vex::percent);
            right.setVelocity(std::abs(out), vex::percent);
            if (out >= 0) {
                left.spin(vex::forward);
                right.spin(vex::forward);
            } else {
                left.spin(vex::reverse);
                right.spin(vex::reverse);
            }
            vex::wait(dt * 1000, vex::msec);
            elapsed += int(dt * 1000);
        }
        left.stop();
        right.stop();
    }

    // Synchronous turn with PID, with optional inline tuning
    void turn(double target, int timeout = 3000,
              double kP = NAN, double kI = NAN, double kD = NAN) {
        const double dt = 0.01;
        int elapsed = 0;
        if (!std::isnan(kP)) turnPID.kP = kP;
        if (!std::isnan(kI)) turnPID.kI = kI;
        if (!std::isnan(kD)) turnPID.kD = kD;
        turnPID.tolerance = 1.0;
        turnPID.reset();
        while (elapsed < timeout) {
            double cur = imu.rotation();
            double error = normalizeAngle(target - cur);
            if (turnPID.atTarget(error)) break;
            double out = turnPID.calculate(error, dt);
            out = std::fmax(std::fmin(out, 100.0), -100.0);
            left.setVelocity(std::abs(out), vex::percent);
            right.setVelocity(std::abs(out), vex::percent);
            if (out >= 0) {
                left.spin(vex::forward);
                right.spin(vex::reverse);
            } else {
                left.spin(vex::reverse);
                right.spin(vex::forward);
            }
            updateOdometry();
            vex::wait(dt * 1000, vex::msec);
            elapsed += int(dt * 1000);
        }
        left.stop();
        right.stop();
    }

    // Synchronous curve to a point (arc/curved path), with optional final heading and inline PID tuning
    void moveToPoint(
        double x, double y, double heading = NAN, double maxSpeed = 100, int timeout = 5000,
        
        double curve_kP = NAN, double curve_kI = NAN, double curve_kD = NAN,
        double drive_kP = NAN, double drive_kI = NAN, double drive_kD = NAN,
        double turn_kP = NAN, double turn_kI = NAN, double turn_kD = NAN
    ) {
        const double dt = 0.01;
        int elapsed = 0;
        // Allow inline PID tuning if values are provided
        if (!std::isnan(curve_kP)) curvePID.kP = curve_kP;
        if (!std::isnan(curve_kI)) curvePID.kI = curve_kI;
        if (!std::isnan(curve_kD)) curvePID.kD = curve_kD;
        if (!std::isnan(drive_kP)) drivePID.kP = drive_kP;
        if (!std::isnan(drive_kI)) drivePID.kI = drive_kI;
        if (!std::isnan(drive_kD)) drivePID.kD = drive_kD;
        if (!std::isnan(turn_kP)) turnPID.kP = turn_kP;
        if (!std::isnan(turn_kI)) turnPID.kI = turn_kI;
        if (!std::isnan(turn_kD)) turnPID.kD = turn_kD;
        curvePID.tolerance = 2.0; // slightly looser for less oscillation
        curvePID.reset();
        double startX = odom.x, startY = odom.y;
        bool useHeading = !std::isnan(heading);
        bool pointReached = false;
        int settleCount = 0;
        const int settleCycles = 10; // must be in tolerance for 10 cycles
        while (elapsed < timeout) {
            updateOdometry();
            double dx = x - odom.x;
            double dy = y - odom.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (!pointReached && curvePID.atTarget(dist)) {
                settleCount++;
                if (settleCount >= settleCycles) {
                    pointReached = true;
                    if (!useHeading) break;
                }
            } else if (!pointReached) {
                settleCount = 0;
            }
            double targetHeading = useHeading && pointReached ? heading : std::atan2(dy, dx) * 180.0 / M_PI;
            double headingError = normalizeAngle(targetHeading - odom.heading);
            double driveOut = pointReached ? 0 : drivePID.calculate(dist, dt);
            double turnOut = turnPID.calculate(headingError, dt);
            // Clamp outputs and add minimum speed to avoid stalling
            driveOut = std::fmax(std::fmin(driveOut, maxSpeed), -maxSpeed);
            if (std::abs(driveOut) > 0 && std::abs(driveOut) < 8) driveOut = (driveOut > 0) ? 8 : -8;
            turnOut = std::fmax(std::fmin(turnOut, 40.0), -40.0);
            if (std::abs(turnOut) > 0 && std::abs(turnOut) < 6) turnOut = (turnOut > 0) ? 6 : -6;
            double leftOut = driveOut + turnOut;
            double rightOut = driveOut - turnOut;
            left.setVelocity(std::abs(leftOut), vex::percent);
            right.setVelocity(std::abs(rightOut), vex::percent);
            if (leftOut >= 0) left.spin(vex::forward); else left.spin(vex::reverse);
            if (rightOut >= 0) right.spin(vex::forward); else right.spin(vex::reverse);
            if (pointReached && useHeading && std::abs(headingError) < 1.0) break;
            vex::wait(dt * 1000, vex::msec);
            elapsed += int(dt * 1000);
        }
        left.stop();
        right.stop();
    }

    // Utility: normalize angle to [-180,180]
    static double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
};

#endif // CHASSIS_H
