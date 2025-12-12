/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module: main.cpp                                                        */
/*    Created: 10/9/2025                                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
using namespace vex;

// A global instance of competition
competition Competition;

// --------------------
// GLOBALS
// --------------------
vex::brain Brain;
vex::controller Controller1;

// Front Motors
vex::motor leftFront(vex::PORT5, ratio6_1, false);
vex::motor rightFront(vex::PORT10, ratio6_1, false);

// Back Motors
vex::motor leftBack(vex::PORT3, ratio6_1, false);
vex::motor rightBack(vex::PORT7, ratio6_1, false);

// Top Motors
vex::motor leftTop(vex::PORT4, ratio6_1, true);
vex::motor rightTop(vex::PORT8, ratio6_1, true);

// Motor Groups
vex::motor_group rightSide(rightFront, rightBack, rightTop);
vex::motor_group leftSide(leftFront, leftBack, leftTop);

// Drivetrain
vex::drivetrain Drivetrain(rightSide, leftSide, 259.34, 320, 40, mm, 1);

// Motors
vex::motor intakeMotor(vex::PORT1, ratio6_1, false);
vex::motor outputMotor(vex::PORT6, ratio6_1, false);

// Pneumatics (use brain three-wire ports)
digital_out Blocker(Brain.ThreeWirePort.A);
digital_out MatchLoader(Brain.ThreeWirePort.B);
digital_out Pistons(Brain.ThreeWirePort.C);

// Sensors
vex::inertial inertialSensor = vex::inertial(PORT21);
vex::distance DistanceSensor = vex::distance(PORT9);

// --------------------
// CALLBACK HANDLERS
// --------------------
// Intake callbacks (R1 = forward, R2 = reverse)
void intakeForwardPressed() { 
  intakeMotor.spin(forward); 
}
void intakeForwardReleased() { 
  if (!Controller1.ButtonR2.pressing()) {
    intakeMotor.stop(); 
  }
}
void intakeReversePressed() { 
  intakeMotor.spin(reverse); 
}
void intakeReverseReleased() { 
  if (!Controller1.ButtonR1.pressing()) 
    intakeMotor.stop(); 
}

// Scoring/output callbacks (L1 = forward, L2 = reverse)
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

// Pneumatic states
bool blockerExtended = false;
bool matchLoaderExtended = false;
bool pistonsExtended = false;

// Blocker (Brain ThreeWire Port A)
void blockerToggle() {
  blockerExtended = !blockerExtended;
  Blocker.set(blockerExtended);
  Brain.Screen.clearLine(2);
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Blocker: %s", blockerExtended ? "EXT" : "CL");
}

// MatchLoader (Brain ThreeWire Port B)
void matchLoaderToggle() {
  matchLoaderExtended = !matchLoaderExtended;
  MatchLoader.set(matchLoaderExtended);
  Brain.Screen.clearLine(4);
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("MatchLD: %s", matchLoaderExtended ? "EXT" : "CL");
}

// Pistons (Brain ThreeWire Port C)
void pistonsToggle() {
  pistonsExtended = !pistonsExtended;
  Pistons.set(pistonsExtended);
  Brain.Screen.clearLine(5);
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("Pistons: %s", pistonsExtended ? "EXT" : "CL");
}

// Deadzone
const int DEADZONE_THRESHOLD = 5;

// Simple slew-rate limiter for doubles
static double slewTo(double target, double current, double maxDelta) {
  double diff = target - current;
  // If current is zero, behave normally
  if (current == 0.0) {
    if (std::abs(diff) <= maxDelta) return target;
    return current + (diff > 0.0 ? maxDelta : -maxDelta);
  }

  // If current and target are same sign (or target is zero), move toward target
  if ((current > 0.0 && target >= 0.0) || (current < 0.0 && target <= 0.0)) {
    if (std::abs(diff) <= maxDelta) return target;
    return current + (diff > 0.0 ? maxDelta : -maxDelta);
  }

  // Target has opposite sign: decelerate toward zero without crossing in one step
  if (std::abs(current) <= maxDelta) return 0.0;
  return current + (current > 0.0 ? -maxDelta : maxDelta);
}

// Runtime-configurable slew delta (percent per control loop iteration)
double slewMaxDelta = 6.0;

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

// Turn sensitivity (percent). 100 = normal
int turnSensitivity = 32;

// Deceleration step when controller released (percent per loop)
const double DECEL_STEP = 1.5;

// --------------------
// MODE SELECTOR
// --------------------
bool skillsMode = false;  
// false = Match Autonomous
// true  = Skills Autonomous

// --------------------
// AUTON FUNCTIONS
// --------------------
void matchAuton() {
  // Drive forward until the distance sensor reads less than 18.75 inches,
  // then stop and run the intake to outtake/score.
  Brain.Screen.clearLine(3);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Rotation: %.1f degrees", inertialSensor.rotation());
  leftSide.setVelocity(20, percent);
  rightSide.setVelocity(20, percent);
  leftSide.spin(reverse);
  rightSide.spin(forward);

  // Wait until an object is detected closer than threshold
  while (DistanceSensor.objectDistance(vex::distanceUnits::in) > 18.75) {
    double dist_in = DistanceSensor.objectDistance(vex::distanceUnits::in);
    Brain.Screen.clearLine(6);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Dist: %.1f in", dist_in);
    wait(20, msec);
  }

  leftSide.stop();
  rightSide.stop(); 
  wait(200, msec);
  intakeMotor.spin(reverse);
  wait(1500, msec);
  intakeMotor.stop();

  Drivetrain.turn(left);
  // while (inertialSensor.rotation())
}

void skillsAuton() {
  /* 
  >>> YOUR SKILLS AUTON CODE HERE <<< 
  */
  double objDist = DistanceSensor.objectDistance(inches);
  Drivetrain.drive(forward);
  while (objDist < 16.5) wait(5, msec); 
  Drivetrain.stop();
  matchLoaderToggle();
  intakeMotor.spin(reverse);
  wait(1500, msec);
  intakeMotor.stop();
  matchLoaderToggle();
  Drivetrain.turn(left);
  while (inertialSensor.rotation() > -30) wait(1, msec);
  Drivetrain.stop();
  Drivetrain.drive(fwd);
  while (objDist < 30) wait(5, msec);
  Drivetrain.stop();
  intakeMotor.spin(forward);
  wait(1500, msec);
  intakeMotor.stop();
  Drivetrain.drive(reverse);
  while (objDist > 20) wait(5, msec); 
  Drivetrain.stop();
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
    wait(50, msec);
  }
  Brain.Screen.clearLine(1);
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("IMU Ready");
  wait(200, msec);
}

void pre_auton(void) {
  // Show mode selection on the Brain
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Mode: %s", skillsMode ? "SKILLS" : "MATCH");

  // Set default stopping modes and bind controller callbacks
  leftSide.setStopping(vex::brakeType::coast);
  rightSide.setStopping(vex::brakeType::coast);
  intakeMotor.setStopping(vex::brakeType::coast);
  outputMotor.setStopping(vex::brakeType::coast);

  // Set sane default velocities for intake/output
  intakeMotor.setVelocity(100, percent);
  outputMotor.setVelocity(100, percent);

  // Calibrate inertial sensor (may take a second)
  calibrateIMU();

  Controller1.ButtonR1.pressed(intakeReversePressed);
  Controller1.ButtonR1.released(intakeReverseReleased);
  Controller1.ButtonR2.pressed(intakeForwardPressed);
  Controller1.ButtonR2.released(intakeForwardReleased);

  Controller1.ButtonL1.pressed(outputForwardPressed);
  Controller1.ButtonL1.released(outputForwardReleased);
  Controller1.ButtonL2.pressed(outputReversePressed);
  Controller1.ButtonL2.released(outputReverseReleased);

  // Pneumatics callbacks
  Controller1.ButtonA.pressed(blockerToggle);
  Controller1.ButtonB.pressed(matchLoaderToggle);
  Controller1.ButtonX.pressed(pistonsToggle);
}

// --------------------
// AUTONOMOUS
// --------------------
void autonomous(void) {
  // Ensure IMU is calibrated before autonomous
  calibrateIMU();
  if (skillsMode) skillsAuton();
  else matchAuton();
}

// --------------------
// DRIVER CONTROL
// --------------------
void usercontrol(void) {
  // Re-calibrate IMU each time usercontrol starts to ensure fresh data
  calibrateIMU();

  double currentLeft = 0.0;
  double currentRight = 0.0;

  while (1) {
    // TOGGLE AUTON MODE USING “LEFT” BUTTON
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

    // Scale turning sensitivity (user-adjustable)
    int scaledTurn = (turnPower * turnSensitivity) / 100;

    int leftOutput = drivePower + scaledTurn;
    int rightOutput = drivePower - scaledTurn;

    // If both sticks are released, decelerate smoothly by DECEL_STEP each loop
    if (drivePower == 0 && turnPower == 0) {
      if (currentLeft > 0.0) { currentLeft -= DECEL_STEP; if (currentLeft < 0.0) currentLeft = 0.0; }
      else if (currentLeft < 0.0) { currentLeft += DECEL_STEP; if (currentLeft > 0.0) currentLeft = 0.0; }

      if (currentRight > 0.0) { currentRight -= DECEL_STEP; if (currentRight < 0.0) currentRight = 0.0; }
      else if (currentRight < 0.0) { currentRight += DECEL_STEP; if (currentRight > 0.0) currentRight = 0.0; }
    } else {
      // Use raw joystick outputs as targets (no input curve)
      double targetLeft = (double)leftOutput;
      double targetRight = (double)rightOutput;

      // Slew toward targets to avoid jumps
      currentLeft = slewTo(targetLeft, currentLeft, slewMaxDelta);
      currentRight = slewTo(targetRight, currentRight, slewMaxDelta);
    }

    // Set velocities using absolute values and choose spin direction
    leftSide.setVelocity(std::abs(currentLeft), percent);
    rightSide.setVelocity(std::abs(currentRight), percent);

    leftSide.spin(currentLeft >= 0.0 ? reverse : forward);
    rightSide.spin(currentRight >= 0.0 ? forward : reverse);

    // Telemetry for debugging turning behavior
    Brain.Screen.clearLine(5);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("T:%d L:%.1f R:%.1f", scaledTurn, currentLeft, currentRight);

    // Show distance sensor (inches) and IMU heading (degrees)
    double dist_in = DistanceSensor.objectDistance(vex::distanceUnits::in);
    double raw_heading = inertialSensor.heading();
    double heading_deg = fmod(raw_heading, 360.0);
    if (heading_deg < 0) heading_deg += 360.0;
    Brain.Screen.clearLine(6);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Dist: %.1f in  H: %.1f deg", dist_in, heading_deg);

    wait(20, msec);
  }
}

// --------------------
// MAIN
// --------------------
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) wait(100, msec);
}
