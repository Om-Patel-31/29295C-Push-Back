#ifndef CHASSIS_H
#define CHASSIS_H

#include "PIDController.h"
#include "OdometryTracker.h"
#include "vex.h"
#include <cmath>

class Chassis {
    // Follow a Hermite curve using PID and odometry
    // Parameters: startX, startY, startHeading, endX, endY, endHeading, tightness, numPoints
public:
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
    vex::motor_group& left;
    vex::motor_group& right;
    vex::inertial& imu;
    OdometryTracker odom;
    PIDController drivePID;
    PIDController turnPID;

    Chassis(vex::motor_group& l, vex::motor_group& r, vex::inertial& i)
        : left(l), right(r), imu(i),
          drivePID(2, 0.5, 0.01),
          turnPID(0.5, 0.01, 0.05) {}

    void reset() {
        left.resetPosition();
        right.resetPosition();
        odom.reset();
        drivePID.reset();
        turnPID.reset();
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
