#include "vex.h"
#include "PIDController.h"
#include "OdometryTracker.h"
#include "DriverControl.h"
#include <cmath>
using namespace vex;

competition Competition;

// DRIVER CONTROL CONFIGURATION - Change this to switch control schemes
DriverControl driverControl(ControlType::ARCADE, 10, 2.0, 60, 50);

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

PIDController drivePID(3, 0.5, 0.01);
PIDController turnPID(0.5, 0.01, 0.05);
OdometryTracker odometry;

// Forward declarations for functions defined later
void updateOdometry();
void calibrateIMU();
extern bool matchAutonSide;

const int IMU_distance = 13;

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

template<typename T>
static T clamp(T value, T minVal, T maxVal) {
    if (value > maxVal) return maxVal;
    if (value < minVal) return minVal;
    return value;
}

void increaseExpo() {
    double currentExpo = driverControl.getExponent();
    if (currentExpo < 5.0) {
      driverControl.setExponent(std::round((currentExpo + 0.1) * 100.0) / 100.0);
      Controller1.Screen.clearLine(3);
      Controller1.Screen.setCursor(3, 1);
      Controller1.Screen.print("Expo: %.2f", driverControl.getExponent());
    }
  }

  void decreaseExpo() {
    double currentExpo = driverControl.getExponent();
    if (currentExpo > 1.0) {
      driverControl.setExponent(std::round((currentExpo - 0.1) * 100.0) / 100.0);
      Controller1.Screen.clearLine(3);
      Controller1.Screen.setCursor(3, 1);
      Controller1.Screen.print("Expo: %.2f", driverControl.getExponent());
    }
  }

  void increaseTurnSens() {
    int currentSens = driverControl.getTurnSensitivity();
    if (currentSens < 100) {
      driverControl.setTurnSensitivity(currentSens + 5);
      Brain.Screen.clearLine(3);
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("Turn Sens: %d", driverControl.getTurnSensitivity());
    }
  }

  void decreaseTurnSens() {
    int currentSens = driverControl.getTurnSensitivity();
    if (currentSens > 10) {
      driverControl.setTurnSensitivity(currentSens - 5);
      Brain.Screen.clearLine(3);
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("Turn Sens: %d", driverControl.getTurnSensitivity());
    }
}

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

bool skillsMode = true;
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
  // Generated VEX C++ code for skillsAuton()
  // Copy and paste this into your skillsAuton() function

  // Starting position: (27.70", 88.00") @ 0°
  // IMU offset from center: 13.00" (forward)
  // Calibration offsets: X=4.00", Y=0.50"
  calibrateIMU();
  resetEncoders();
  odometry.reset();
  drivePID.reset();
  turnPID.reset();

  // Path 1
  // Waypoint 1: (22.98", 118.83") - Distance: 31.34" (IMU: 52.06")
  driveWithPID(31.34 + IMU_distance);
  updateOdometry();

  Controller1.rumble(".−");
  // End of generated code
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

  double leftOutput = 0.0;
  double rightOutput = 0.0;
  int lastTelemetryMs = Brain.timer(msec);

  while (1) {
    // Toggle between skills and match mode
    if (Controller1.ButtonLeft.pressing()) {
      skillsMode = !skillsMode;
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Mode: %s", skillsMode ? "SKILLS" : "MATCH");
      wait(300, msec);
    }

    // Get driver control outputs (ButtonY for slow turn mode)
    bool slowTurn = Controller1.ButtonY.pressing();
    driverControl.calculate(Controller1, leftOutput, rightOutput, slowTurn);

    leftSide.setVelocity(std::abs(leftOutput), percent);
    rightSide.setVelocity(std::abs(rightOutput), percent);
    
    if (std::abs(leftOutput) < 0.5 && std::abs(rightOutput) < 0.5) {
      leftSide.stop(coast);
      rightSide.stop(coast);
    } else {
      leftSide.spin(leftOutput >= 0.0 ? forward : reverse);
      rightSide.spin(rightOutput >= 0.0 ? forward : reverse);
    }

    // Optional telemetry display every 100ms
    int currentMs = Brain.timer(msec);
    if (currentMs - lastTelemetryMs > 100) {
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("L:%.1f R:%.1f   ", leftOutput, rightOutput);
      lastTelemetryMs = currentMs;
    }

    wait(20, msec);
  }
}

int main() {
  pre_auton();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true) {
    wait(100, msec);
  }
}
