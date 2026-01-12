#include "vex.h"
#include <cmath>
using namespace vex;

competition Competition;

class PIDController {
public:
  double kP, kI, kD;
  double integral = 0.0;
  double prevError = 0.0;
  double maxIntegral = 50.0;
  double tolerance = 1.0;
  bool enabled = false;
  double lastP = 0.0;
  double lastI = 0.0;
  double lastD = 0.0;

  PIDController(double p, double i, double d) : kP(p), kI(i), kD(d) {}

  double calculate(double error, double dt) {
    lastP = kP * error;

    integral += error * dt;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;
    lastI = kI * integral;

    lastD = 0.0;
    if (dt > 0) {
      lastD = kD * (error - prevError) / dt;
    }

    prevError = error;

    return lastP + lastI + lastD;
  }

  void reset() {
    integral = 0.0;
    prevError = 0.0;
  }

  bool atTarget(double error) {
    return std::abs(error) < tolerance;
  }
};

class OdometryTracker {
public:
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double wheelBase = 10;
  double trackingWheelRadius = 3.25;

  void update(double leftDist, double rightDist, double imuHeading) {
    double avgDist = (leftDist + rightDist) / 2.0;

    heading = imuHeading;

    double headingRad = heading * M_PI / 180.0;
    x += avgDist * std::cos(headingRad);
    y += avgDist * std::sin(headingRad);
  }

  void reset() {
    x = 0.0;
    y = 0.0;
    heading = 0.0;
  }

  void setPosition(double newX, double newY, double newHeading) {
    x = newX;
    y = newY;
    heading = newHeading;
  }

  double distanceTo(double targetX, double targetY) {
    double dx = targetX - x;
    double dy = targetY - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double headingTo(double targetX, double targetY) {
    double dx = targetX - x;
    double dy = targetY - y;
    return std::atan2(dy, dx) * 180.0 / M_PI;
  }
};

vex::brain Brain;
vex::controller Controller1;

vex::motor leftFront(vex::PORT5, ratio6_1, true);
vex::motor rightFront(vex::PORT10, ratio6_1, false);

vex::motor leftBack(vex::PORT3, ratio6_1, true);
vex::motor rightBack(vex::PORT7, ratio6_1, false);

vex::motor leftTop(vex::PORT4, ratio6_1, false);
vex::motor rightTop(vex::PORT8, ratio6_1, true);

vex::motor_group rightSide(rightFront, rightBack, rightTop);
vex::motor_group leftSide(leftFront, leftBack, leftTop);

vex::drivetrain Drivetrain(leftSide, rightSide, 259.34, 320, 40, mm, 1);

vex::motor intakeMotor(vex::PORT1, ratio6_1, true);
vex::motor outputMotor(vex::PORT6, ratio6_1, false);

digital_out Blocker(Brain.ThreeWirePort.A);
digital_out MatchLoader(Brain.ThreeWirePort.B);
digital_out Pistons(Brain.ThreeWirePort.C);

vex::inertial inertialSensor = vex::inertial(PORT21);

PIDController drivePID(3, 0.05, 0.00001);
PIDController turnPID(0.5, 0.01, 0.05);
OdometryTracker odometry;

// Forward declarations for functions defined later
void updateOdometry();
void calibrateIMU();
extern bool matchAutonSide;

const int IMU_distance = 8;

void intakeForwardPressed() { 
    intakeMotor.spin(reverse); 
  }
  void intakeForwardReleased() { 
    if (!Controller1.ButtonR2.pressing()) {
      intakeMotor.stop(); 
    }
  }
  void intakeReversePressed() { 
    intakeMotor.spin(forward); 
  }
  void intakeReverseReleased() { 
    if (!Controller1.ButtonR1.pressing()) 
      intakeMotor.stop(); 
  }

void outputForwardPressed() { 
    outputMotor.spin(forward); 
  }
  void outputForwardReleased() { 
    if (!Controller1.ButtonL2.pressing()) {
      outputMotor.stop(); 
    }
  }
  void outputReversePressed() {
    outputMotor.spin(reverse); 
  }
  void outputReverseReleased() { 
    if (!Controller1.ButtonL1.pressing()) {
      outputMotor.stop(); 
    }
  }

  void SpinBothForwardPressed() {
    intakeForwardPressed();
    outputForwardPressed();
  }

  void SpinBothForwardReleased() {
    intakeForwardReleased();
    outputForwardReleased();
  }

  void SpinBothReversePressed() {
    intakeReversePressed();
    outputReversePressed();
  }

  void SpinBothReverseReleased() {
    intakeReverseReleased();
    outputReverseReleased();
  }

bool blockerExtended = false;
  bool matchLoaderExtended = false;
  bool pistonsExtended = false;

void blockerToggle() {
    blockerExtended = !blockerExtended;
    Blocker.set(blockerExtended);
    Brain.Screen.clearLine(2);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Blocker: %s", blockerExtended ? "EXT" : "CL");
  }

void matchLoaderToggle() {
    matchLoaderExtended = !matchLoaderExtended;
    MatchLoader.set(matchLoaderExtended);
    Brain.Screen.clearLine(4);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("MatchLD: %s", matchLoaderExtended ? "EXT" : "CL");
  }

void pistonsToggle() {
    pistonsExtended = !pistonsExtended;
    Pistons.set(pistonsExtended);
    Brain.Screen.clearLine(5);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Pistons: %s", pistonsExtended ? "EXT" : "CL");
  }

// Auton side selector handlers (Up = RIGHT, Down = LEFT)
void selectRightAuton() {
  matchAutonSide = true;
  Brain.Screen.clearLine(2);
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Auton: RIGHT");
}

void selectLeftAuton() {
  matchAutonSide = false;
  Brain.Screen.clearLine(2);
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Auton: LEFT");
}

const int DEADZONE_THRESHOLD = 10;

static double slewTo(double target, double current, double maxDelta) {
  double diff = target - current;
  if (current == 0.0) {
    if (std::abs(diff) <= maxDelta) return target;
    return current + (diff > 0.0 ? maxDelta : -maxDelta);
  }

  if ((current > 0.0 && target >= 0.0) || (current < 0.0 && target <= 0.0)) {
    if (std::abs(diff) <= maxDelta) return target;
    return current + (diff > 0.0 ? maxDelta : -maxDelta);
  }

  if (std::abs(current) <= maxDelta) return 0.0;
  return current + (current > 0.0 ? -maxDelta : maxDelta);
}

double slewMaxDelta = 7.50;

template<typename T>
static T clamp(T value, T minVal, T maxVal) {
    if (value > maxVal) return maxVal;
    if (value < minVal) return minVal;
    return value;
}

static double clampPercent(double v) {
    return clamp(v, -100.0, 100.0);
}

static double expoCurve(double inputPercent, double exponent) {
  double v = clampPercent(inputPercent);
  double sign = (v >= 0.0) ? 1.0 : -1.0;
  double absNorm = std::abs(v) / 100.0;
  double shaped = std::pow(absNorm, exponent) * 100.0;
  return sign * shaped;
}

double INPUT_EXPO = 2.0;

void increaseExpo() {
    if (INPUT_EXPO < 5.0) INPUT_EXPO = std::round((INPUT_EXPO + 0.1) * 100.0) / 100.0;
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Expo: %.2f", INPUT_EXPO);
  }

  void decreaseExpo() {
    if (INPUT_EXPO > 1.0) INPUT_EXPO = std::round((INPUT_EXPO - 0.1) * 100.0) / 100.0;
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Expo: %.2f", INPUT_EXPO);
  }

  void increaseSlew() {
    if (slewMaxDelta < 50.0) slewMaxDelta++;
    Brain.Screen.clearLine(3);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Slew: %.0f", slewMaxDelta);
  }

  void decreaseSlew() {
    if (slewMaxDelta > 0.0) slewMaxDelta--;
    Brain.Screen.clearLine(3);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Slew: %.0f", slewMaxDelta);
}

int turnSensitivity = 60;
const int slowTurnSensitivity = 50;

void driveWithPID(double targetDist, int velocity = 100, int timeout = 5000, bool useDistanceSensor = false) {
  drivePID.reset();
  drivePID.tolerance = 0.1;
  
  double leftStartPos = leftSide.position(vex::rotationUnits::rev) * 10.2;
  double rightStartPos = rightSide.position(vex::rotationUnits::rev) * 10.2;
    double elapsedTime = 0;
    
    while (elapsedTime < timeout) {
      double leftCurr = leftSide.position(rev) * 10.2;
      double rightCurr = rightSide.position(rev) * 10.2;
      
      double leftDist = leftCurr - leftStartPos;
      double rightDist = rightCurr - rightStartPos;
    double avgDist = (leftDist + rightDist) / 2.0;
    
    double error = targetDist - avgDist;
    
    if (drivePID.atTarget(error)) {
      leftSide.stop(brake);
      rightSide.stop(brake);
        break;
    }
    
    double pidOutput = drivePID.calculate(error, 0.01);
    double driveSpeed = clamp(pidOutput, -static_cast<double>(velocity), static_cast<double>(velocity));
      
      leftSide.setVelocity(std::abs(driveSpeed), percent);
      rightSide.setVelocity(std::abs(driveSpeed), percent);
      
      if (driveSpeed >= 0) {
        leftSide.spin(forward);
        rightSide.spin(forward);
      } else {
        leftSide.spin(reverse);
        rightSide.spin(reverse);
      }
      // Update odometry at 1 ms intervals during auton moves
      updateOdometry();
      wait(1, msec);
      elapsedTime += 1;
    }
    
    leftSide.stop();
    rightSide.stop();
}

void turnWithPID(double targetHeading, int timeout = 3000) {
  turnPID.reset();
  turnPID.tolerance = 1.0;
    
    double elapsedTime = 0;
    
    while (elapsedTime < timeout) {
    double currentHeading = inertialSensor.rotation();
    double error = targetHeading - currentHeading;
    
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
      
      if (turnPID.atTarget(error)) {
        leftSide.stop();
        rightSide.stop();
        break;
      }
      
      double pidOutput = turnPID.calculate(error, 0.01);
      double turnSpeed = clamp(pidOutput, -100.0, 100.0);
      
      leftSide.setVelocity(std::abs(turnSpeed), percent);
      rightSide.setVelocity(std::abs(turnSpeed), percent);
      
      if (turnSpeed >= 0) {
        leftSide.spin(forward);
        rightSide.spin(reverse);
      } else {
        leftSide.spin(reverse);
        rightSide.spin(forward);
      }
      // Update odometry at 1 ms intervals during auton turns
      updateOdometry();
      wait(1, msec);
      elapsedTime += 1;
    }
    
    leftSide.stop();
    rightSide.stop();
}

void updateOdometry() {
  double currentHeading = inertialSensor.rotation();
  double leftDist = leftSide.position(vex::rotationUnits::rev) * 10.2;
  double rightDist = rightSide.position(vex::rotationUnits::rev) * 10.2;
    odometry.update(leftDist, rightDist, currentHeading);
}

void resetEncoders() {
  leftSide.resetPosition();
    rightSide.resetPosition();
    intakeMotor.resetPosition();
    outputMotor.resetPosition();
}

bool skillsMode = false;
bool matchAutonSide = false;
bool autonTestActive = false;

  // --------------------
  // AUTON FUNCTIONS
  // --------------------
  // Forward declaration for skills autonomous
void skillsAuton();

void matchAutonRight() {
  calibrateIMU();
  Brain.Screen.clearLine(3);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Match Auton: RIGHT");
  
  resetEncoders();
  odometry.reset();
  drivePID.reset();
  turnPID.reset();

  matchLoaderToggle();
  
  intakeMotor.spin(forward);
  driveWithPID(std::sqrt(2275.0) - IMU_distance);
  updateOdometry();
  
  turnWithPID(20);
  driveWithPID(5);
  
  intakeMotor.spin(forward);
  matchLoaderToggle();
  driveWithPID(5);
  intakeMotor.spin(forward);
  wait(4000, msec);
  matchLoaderToggle();
  
  driveWithPID(8);
  turnWithPID(-40);
  
  driveWithPID(14);
  outputMotor.spin(reverse);
  intakeMotor.spin(reverse);
  wait(5000, msec);
  
  Controller1.rumble(".-");
  }

void matchAutonLeft() {
  Brain.Screen.clearLine(3);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Match Auton: LEFT");
  
  resetEncoders();
  odometry.reset();
  drivePID.reset();
  turnPID.reset();

  matchLoaderToggle();
  
  intakeMotor.spin(forward);
  driveWithPID(std::sqrt(2275.0) - IMU_distance);
  updateOdometry();
  
  turnWithPID(-20);
  driveWithPID(5);
  
  intakeMotor.spin(forward);
  matchLoaderToggle();
  driveWithPID(5);
  intakeMotor.spin(forward);
  wait(3000, msec);
  matchLoaderToggle();
  
  driveWithPID(8);
  turnWithPID(45);
  blockerToggle();
  
  driveWithPID(14);
  outputMotor.spin(forward);
  intakeMotor.spin(forward);
  wait(5000, msec);
  }

void skillsAuton() {
  // Starting position: (26.40", 87.60") @ 0°
  calibrateIMU();
  resetEncoders();
  odometry.reset();
  drivePID.reset();
  turnPID.reset();

  // Path 1
  // Waypoint 1: (26.16", 119.52") - Distance: 31.92"
  turnWithPID(-89.6);
  driveWithPID(31.92);
  updateOdometry();

  // Waypoint 2: (4.32", 119.76") - Distance: 21.84"
  turnWithPID(-0.6);
  driveWithPID(21.84);
  updateOdometry();

  // Waypoint 3: (15.36", 119.76") - Distance: 11.04"
  turnWithPID(180.0);
  driveWithPID(11.04);
  updateOdometry();

  // Waypoint 4: (26.88", 131.76") - Distance: 16.63"
  turnWithPID(-133.8);
  driveWithPID(16.63);
  updateOdometry();

  // Waypoint 5: (117.36", 132.00") - Distance: 90.48"
  turnWithPID(-179.8);
  driveWithPID(90.48);
  updateOdometry();

  // Waypoint 6: (117.36", 119.76") - Distance: 12.24"
  turnWithPID(90.0);
  driveWithPID(12.24);
  updateOdometry();

  // Waypoint 7: (98.88", 119.76") - Distance: 18.48"
  turnWithPID(0.0);
  driveWithPID(18.48);
  updateOdometry();

  // Waypoint 8: (140.64", 120.00") - Distance: 41.76"
  turnWithPID(-179.7);
  driveWithPID(41.76);
  updateOdometry();

  // Waypoint 9: (117.36", 120.00") - Distance: 23.28"
  turnWithPID(0.0);
  driveWithPID(23.28);
  updateOdometry();

  // Waypoint 10: (117.60", 27.12") - Distance: 92.88"
  turnWithPID(90.1);
  driveWithPID(92.88);
  updateOdometry();

  // Waypoint 11: (136.56", 27.12") - Distance: 18.96"
  turnWithPID(180.0);
  driveWithPID(18.96);
  updateOdometry();

  // Waypoint 12: (127.68", 27.12") - Distance: 8.88"
  turnWithPID(0.0);
  driveWithPID(8.88);
  updateOdometry();

  // Waypoint 13: (117.12", 16.32") - Distance: 15.10"
  turnWithPID(45.6);
  driveWithPID(15.10);
  updateOdometry();

  // Waypoint 14: (26.88", 16.32") - Distance: 90.24"
  turnWithPID(0.0);
  driveWithPID(90.24);
  updateOdometry();

  // Waypoint 15: (26.88", 27.60") - Distance: 11.28"
  turnWithPID(-90.0);
  driveWithPID(11.28);
  updateOdometry();

  // Waypoint 16: (43.44", 27.60") - Distance: 16.56"
  turnWithPID(180.0);
  driveWithPID(16.56);
  updateOdometry();

  // Waypoint 17: (8.40", 27.84") - Distance: 35.04"
  turnWithPID(-0.4);
  driveWithPID(35.04);
  updateOdometry();

  // Waypoint 18: (43.44", 27.60") - Distance: 35.04"
  turnWithPID(179.6);
  driveWithPID(35.04);
  updateOdometry();

  // Waypoint 19: (7.92", 48.00") - Distance: 40.96"
  turnWithPID(-29.9);
  driveWithPID(40.96);
  updateOdometry();

  // Waypoint 20: (7.92", 78.72") - Distance: 30.72"
  turnWithPID(-90.0);
  driveWithPID(30.72);
  updateOdometry();

  Controller1.rumble(".−");
}

// --------------------
  // PRE-AUTON
  // --------------------
  void calibrateIMU() {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibrating IMU...");
    inertialSensor.calibrate();
    while (inertialSensor.isCalibrating()) {
      wait(5, msec);
    }
    Brain.Screen.clearLine(1);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("IMU Ready");
    Controller1.Screen.print("IMU Ready");
    wait(20, msec);
  }

void pre_auton(void) {
  // Force default to LEFT auton at startup
  matchAutonSide = false;
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Mode: %s", skillsMode ? "SKILLS" : "MATCH");
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Auton: %s", matchAutonSide ? "RIGHT" : "LEFT");

  leftSide.setStopping(vex::brakeType::coast);
  rightSide.setStopping(vex::brakeType::coast);
  intakeMotor.setStopping(vex::brakeType::brake);
  outputMotor.setStopping(vex::brakeType::brake);

  intakeMotor.setVelocity(100, percent);
  outputMotor.setVelocity(100, percent);

  calibrateIMU();

    Controller1.ButtonR1.pressed(intakeReversePressed);
    Controller1.ButtonR1.released(intakeReverseReleased);
    Controller1.ButtonR2.pressed(intakeForwardPressed);
    Controller1.ButtonR2.released(intakeForwardReleased);

    Controller1.ButtonL1.pressed(outputForwardPressed);
    Controller1.ButtonL1.released(outputForwardReleased);
    Controller1.ButtonL2.pressed(outputReversePressed);
  Controller1.ButtonL2.released(outputReverseReleased);

  Controller1.ButtonA.pressed(blockerToggle);
    Controller1.ButtonB.pressed(matchLoaderToggle);
    Controller1.ButtonX.pressed(pistonsToggle);
    Controller1.ButtonUp.pressed(selectRightAuton);
    Controller1.ButtonDown.pressed(selectLeftAuton);
}

void autonomous(void) {
  calibrateIMU();
    if (skillsMode) skillsAuton();
    else if (matchAutonSide) matchAutonRight();
    else matchAutonLeft();
}

void usercontrol(void) {

  // IMU already calibrated in pre_auton, no need to recalibrate
  odometry.reset();

    double currentLeft = 0.0;
    double currentRight = 0.0;
    double currentDrive = 0.0;
    int lastTelemetryMs = Brain.timer(msec);

    
  while (1) {
    // Distance sensor removed; no longer printing its readings
    if (Controller1.ButtonLeft.pressing()) {
        skillsMode = !skillsMode;
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Mode: %s", skillsMode ? "SKILLS" : "MATCH");
        wait(300, msec);
      }

      int drivePower = Controller1.Axis3.position(percent);
      int turnPower = Controller1.Axis1.position(percent);

    if (std::abs(drivePower) < DEADZONE_THRESHOLD) drivePower = 0;
    if (std::abs(turnPower) < DEADZONE_THRESHOLD) turnPower = 0;

    int activeTurnSens = Controller1.ButtonY.pressing() ? slowTurnSensitivity : turnSensitivity;
    int scaledTurn = (turnPower * activeTurnSens) / 100;

      // Apply expo separately to drive and turn
      double targetDrive = expoCurve((double)drivePower, INPUT_EXPO);
      double targetTurn = expoCurve((double)scaledTurn, INPUT_EXPO);

      // Direct response - no slew limiting for instant control
      currentDrive = targetDrive;

      // Compose final motor targets for instant response
      double targetLeft = clampPercent(currentDrive + targetTurn);
      double targetRight = clampPercent(currentDrive - targetTurn);

      currentLeft = targetLeft;
      currentRight = targetRight;

    leftSide.setVelocity(std::abs(currentLeft), percent);
    rightSide.setVelocity(std::abs(currentRight), percent);
    
    if (std::abs(currentLeft) < 0.5 && std::abs(currentRight) < 0.5) {
        leftSide.stop(coast);
        rightSide.stop(coast);
      } else {
        leftSide.spin(currentLeft >= 0.0 ? forward : reverse);
        rightSide.spin(currentRight >= 0.0 ? forward : reverse);
    }

    int nowMs = Brain.timer(msec);
    if (nowMs - lastTelemetryMs >= 100) {
      lastTelemetryMs = nowMs;
      updateOdometry();  // Only update odometry every 100ms to avoid lag
      Brain.Screen.setCursor(5, 1);
      Brain.Screen.print("T:%d L:%.1f R:%.1f   ", scaledTurn, currentLeft, currentRight);

      double raw_heading = inertialSensor.heading();
      double heading_deg = fmod(raw_heading, 360.0);
      if (heading_deg < 0) heading_deg += 360.0;
      Brain.Screen.setCursor(6, 1);
      Brain.Screen.print("H: %.1f deg         ", heading_deg);

      Brain.Screen.setCursor(7, 1);
      Brain.Screen.print("X:%.1f Y:%.1f HDG:%.1f   ", odometry.x, odometry.y, odometry.heading);
    }

      wait(5, msec);
  }

}

int main() {
  pre_auton();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
}
