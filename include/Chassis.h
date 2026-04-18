#ifndef CHASSIS_H
#define CHASSIS_H

#include "PIDController.h"
#include "PIDConfig.h"
#include "OdometryTracker.h"
#include "vex.h"
#include <cmath>

class Chassis {
public:
    vex::motor_group& left;
    vex::motor_group& right;
    vex::inertial& imu;
    vex::rotation* leftRotA;
    vex::rotation* leftRotB;
    vex::rotation* rightRotA;
    vex::rotation* rightRotB;
    vex::gps* gpsSensor;
    vex::distance* distanceSensor;

    OdometryTracker odom;
    PIDController drivePID;
    PIDController turnPID;
    PIDController sDriveDistancePID;
    PIDController sDriveHeadingPID;

    // Conversion from rotation-sensor revolutions to linear inches.
    // Keep this value in sync with your wheel diameter / gearing.
    double rotationInchesPerRev = 10.2;
    bool useGPSFusion = false;
    double gpsFusionAlpha = PIDConfig::gpsFusionAlpha;
    bool useDistanceFusion = true;
    double distanceFusionAlpha = PIDConfig::distanceFusionAlpha;

private:
    double prevLeftDistIn = 0.0;
    double prevRightDistIn = 0.0;

    static double clamp(double value, double minVal, double maxVal) {
        if (value < minVal) return minVal;
        if (value > maxVal) return maxVal;
        return value;
    }

    bool hasRequiredOdoSensors() const {
        return leftRotA != nullptr && leftRotB != nullptr &&
               rightRotA != nullptr && rightRotB != nullptr;
    }

    double readLeftDistanceIn() const {
        if (!hasRequiredOdoSensors()) return prevLeftDistIn;
        double a = leftRotA->position(vex::rotationUnits::rev) * rotationInchesPerRev;
        double b = leftRotB->position(vex::rotationUnits::rev) * rotationInchesPerRev;
        return (a + b) / 2.0;
    }

    double readRightDistanceIn() const {
        if (!hasRequiredOdoSensors()) return prevRightDistIn;
        double a = rightRotA->position(vex::rotationUnits::rev) * rotationInchesPerRev;
        double b = rightRotB->position(vex::rotationUnits::rev) * rotationInchesPerRev;
        return (a + b) / 2.0;
    }

    double readDistanceSensorIn() const {
        if (distanceSensor == nullptr) return NAN;
        double reading = distanceSensor->objectDistance(vex::distanceUnits::in);
        if (reading <= 0.0 || reading > PIDConfig::distanceMaxValidIn) return NAN;
        return reading;
    }

    double fuseLinearDistance(double odoLinearDist, double startDistanceIn) const {
        if (!useDistanceFusion) return odoLinearDist;
        double currentDistance = readDistanceSensorIn();
        if (std::isnan(startDistanceIn) || std::isnan(currentDistance)) return odoLinearDist;

        double distanceDelta = startDistanceIn - currentDistance;
        if (std::abs(distanceDelta) < PIDConfig::distanceMinDeltaIn) return odoLinearDist;

        return (1.0 - distanceFusionAlpha) * odoLinearDist + distanceFusionAlpha * distanceDelta;
    }

    void applyTankOutput(double leftOut, double rightOut) {
        leftOut = clamp(leftOut, -100.0, 100.0);
        rightOut = clamp(rightOut, -100.0, 100.0);

        left.setVelocity(std::abs(leftOut), vex::percent);
        right.setVelocity(std::abs(rightOut), vex::percent);
        left.spin((leftOut >= 0.0) ? vex::forward : vex::reverse);
        right.spin((rightOut >= 0.0) ? vex::forward : vex::reverse);
    }

    void fuseWithGPS() {
        if (!useGPSFusion || gpsSensor == nullptr) return;

        const double mmPerIn = 25.4;
        double gpsXIn = gpsSensor->xPosition(vex::distanceUnits::mm) / mmPerIn;
        double gpsYIn = gpsSensor->yPosition(vex::distanceUnits::mm) / mmPerIn;

        odom.x = (1.0 - gpsFusionAlpha) * odom.x + gpsFusionAlpha * gpsXIn;
        odom.y = (1.0 - gpsFusionAlpha) * odom.y + gpsFusionAlpha * gpsYIn;

        double gpsHeading = normalizeAngle(gpsSensor->heading(vex::rotationUnits::deg));
        double headingError = normalizeAngle(gpsHeading - odom.heading);
        odom.heading = normalizeAngle(odom.heading + headingError * gpsFusionAlpha);
    }

public:
    void turn(double target, int timeout = 3000,
              double kP = NAN, double kI = NAN, double kD = NAN) {
        const double dt = 0.01;
        int elapsed = 0;
        if (!std::isnan(kP)) turnPID.kP = kP;
        if (!std::isnan(kI)) turnPID.kI = kI;
        if (!std::isnan(kD)) turnPID.kD = kD;
        turnPID.tolerance = PIDConfig::turnToleranceDeg;
        turnPID.maxIntegral = PIDConfig::maxIntegralTurn;
        turnPID.reset();
        while (elapsed < timeout) {
            double cur = imu.rotation();
            double error = normalizeAngle(target - cur);
            if (turnPID.atTarget(error)) break;
            double out = turnPID.calculate(error, dt);
            out = clamp(out, -100.0, 100.0);
            applyTankOutput(out, -out);
            updateOdometry();
            vex::wait(dt * 1000, vex::msec);
            elapsed += int(dt * 1000);
        }
        left.stop();
        right.stop();
    }

    Chassis(vex::motor_group& l, vex::motor_group& r, vex::inertial& i)
        : left(l), right(r), imu(i),
          leftRotA(nullptr), leftRotB(nullptr), rightRotA(nullptr), rightRotB(nullptr),
          gpsSensor(nullptr),
                    distanceSensor(nullptr),
          drivePID(PIDConfig::drive.kP, PIDConfig::drive.kI, PIDConfig::drive.kD),
          turnPID(PIDConfig::turn.kP, PIDConfig::turn.kI, PIDConfig::turn.kD),
          sDriveDistancePID(PIDConfig::sDriveDistance.kP, PIDConfig::sDriveDistance.kI, PIDConfig::sDriveDistance.kD),
          sDriveHeadingPID(PIDConfig::sDriveHeading.kP, PIDConfig::sDriveHeading.kI, PIDConfig::sDriveHeading.kD) {}

    Chassis(vex::motor_group& l, vex::motor_group& r, vex::inertial& i,
            vex::rotation& lA, vex::rotation& lB,
            vex::rotation& rA, vex::rotation& rB,
                        vex::gps* gps = nullptr,
                        vex::distance* distance = nullptr)
        : left(l), right(r), imu(i),
          leftRotA(&lA), leftRotB(&lB), rightRotA(&rA), rightRotB(&rB),
          gpsSensor(gps),
                    distanceSensor(distance),
          drivePID(PIDConfig::drive.kP, PIDConfig::drive.kI, PIDConfig::drive.kD),
          turnPID(PIDConfig::turn.kP, PIDConfig::turn.kI, PIDConfig::turn.kD),
          sDriveDistancePID(PIDConfig::sDriveDistance.kP, PIDConfig::sDriveDistance.kI, PIDConfig::sDriveDistance.kD),
          sDriveHeadingPID(PIDConfig::sDriveHeading.kP, PIDConfig::sDriveHeading.kI, PIDConfig::sDriveHeading.kD) {}

    void reset() {
        left.resetPosition();
        right.resetPosition();
        if (leftRotA != nullptr) leftRotA->resetPosition();
        if (leftRotB != nullptr) leftRotB->resetPosition();
        if (rightRotA != nullptr) rightRotA->resetPosition();
        if (rightRotB != nullptr) rightRotB->resetPosition();
        odom.reset();
        drivePID.reset();
        turnPID.reset();
        sDriveDistancePID.reset();
        sDriveHeadingPID.reset();
        prevLeftDistIn = 0.0;
        prevRightDistIn = 0.0;
    }

    void setGPSFusion(bool enabled, double alpha = PIDConfig::gpsFusionAlpha) {
        useGPSFusion = enabled;
        gpsFusionAlpha = clamp(alpha, 0.0, 1.0);
    }

    void setDistanceFusion(bool enabled, double alpha = PIDConfig::distanceFusionAlpha) {
        useDistanceFusion = enabled;
        distanceFusionAlpha = clamp(alpha, 0.0, 1.0);
    }

    void updateOdometry() {
        if (!hasRequiredOdoSensors()) return;
        double heading = normalizeAngle(imu.rotation());
        double leftPos = readLeftDistanceIn();
        double rightPos = readRightDistanceIn();
        double leftDelta = leftPos - prevLeftDistIn;
        double rightDelta = rightPos - prevRightDistIn;
        odom.update(leftDelta, rightDelta, heading);
        prevLeftDistIn = leftPos;
        prevRightDistIn = rightPos;
        fuseWithGPS();
    }

    // Synchronous straight drive with PID, with optional inline tuning
    void drive(double dist, double maxSpeed = 100, int timeout = 5000,
               double kP = NAN, double kI = NAN, double kD = NAN) {
        const double dt = 0.01;
        int elapsed = 0;
        double startX = odom.x, startY = odom.y, startH = odom.heading;
        double startDistanceIn = readDistanceSensorIn();
        if (!std::isnan(kP)) drivePID.kP = kP;
        if (!std::isnan(kI)) drivePID.kI = kI;
        if (!std::isnan(kD)) drivePID.kD = kD;
        drivePID.tolerance = PIDConfig::driveToleranceIn;
        drivePID.maxIntegral = PIDConfig::maxIntegralDrive;
        drivePID.reset();
        while (elapsed < timeout) {
            updateOdometry();
            double dx = odom.x - startX;
            double dy = odom.y - startY;
            double headingRad = startH * M_PI / 180.0;
            double odoDist = dx * std::cos(headingRad) + dy * std::sin(headingRad);
            double curDist = fuseLinearDistance(odoDist, startDistanceIn);
            double error = dist - curDist;
            if (drivePID.atTarget(error)) break;
            double out = drivePID.calculate(error, dt);
            out = clamp(out, -maxSpeed, maxSpeed);
            applyTankOutput(out, out);
            vex::wait(dt * 1000, vex::msec);
            elapsed += int(dt * 1000);
        }
        left.stop();
        right.stop();
    }

    // Simultaneous drive + heading control to a 2D point and final heading.
    // Designed for curve-like motion and swing turns when angle error is large.
    void s_drive(double targetX, double targetY, double targetHeading,
                 double maxSpeed = 100, int timeout = 5000,
                 double distkP = NAN, double distkI = NAN, double distkD = NAN,
                 double headkP = NAN, double headkI = NAN, double headkD = NAN,
                 bool allowSwing = true) {
        const double dt = 0.01;
        int elapsed = 0;
        double startDistanceIn = readDistanceSensorIn();

        if (!std::isnan(distkP)) sDriveDistancePID.kP = distkP;
        if (!std::isnan(distkI)) sDriveDistancePID.kI = distkI;
        if (!std::isnan(distkD)) sDriveDistancePID.kD = distkD;

        if (!std::isnan(headkP)) sDriveHeadingPID.kP = headkP;
        if (!std::isnan(headkI)) sDriveHeadingPID.kI = headkI;
        if (!std::isnan(headkD)) sDriveHeadingPID.kD = headkD;

        sDriveDistancePID.tolerance = PIDConfig::sDriveDistanceToleranceIn;
        sDriveHeadingPID.tolerance = PIDConfig::sDriveHeadingToleranceDeg;
        sDriveDistancePID.maxIntegral = PIDConfig::maxIntegralSDriveDistance;
        sDriveHeadingPID.maxIntegral = PIDConfig::maxIntegralSDriveHeading;
        sDriveDistancePID.reset();
        sDriveHeadingPID.reset();

        while (elapsed < timeout) {
            updateOdometry();

            double dx = targetX - odom.x;
            double dy = targetY - odom.y;
            double odoDistance = std::sqrt(dx * dx + dy * dy);

            double pathHeading = std::atan2(dy, dx) * 180.0 / M_PI;
            double headingRad = odom.heading * M_PI / 180.0;
            double forwardProjection = dx * std::cos(headingRad) + dy * std::sin(headingRad);
            double fusedForwardDistance = fuseLinearDistance(forwardProjection, startDistanceIn);
            double distance = (fusedForwardDistance > 0.0)
                ? fusedForwardDistance
                : odoDistance;

            double pathHeadingError = normalizeAngle(pathHeading - odom.heading);
            double finalHeadingError = normalizeAngle(targetHeading - odom.heading);

            double blend = clamp(distance / PIDConfig::sDriveBlendDistanceIn, 0.0, 1.0);
            double headingError = blend * pathHeadingError + (1.0 - blend) * finalHeadingError;

            if (distance < sDriveDistancePID.tolerance &&
                std::abs(finalHeadingError) < sDriveHeadingPID.tolerance) {
                break;
            }

            double driveOut = sDriveDistancePID.calculate(distance, dt);
            double turnOut = sDriveHeadingPID.calculate(headingError, dt);

            driveOut = clamp(driveOut, -maxSpeed, maxSpeed);
            turnOut = clamp(turnOut, -maxSpeed, maxSpeed);

            double leftOut = driveOut + turnOut;
            double rightOut = driveOut - turnOut;

            if (allowSwing && std::abs(headingError) > PIDConfig::swingTurnThresholdDeg) {
                if (headingError > 0.0) {
                    rightOut *= PIDConfig::swingInsideScale;
                } else {
                    leftOut *= PIDConfig::swingInsideScale;
                }
            }

            applyTankOutput(leftOut, rightOut);
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
