/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module: main.cpp                                                        */
/*    Author:                                                                 */
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
vex::brain Brain1;
vex::controller Controller1;

// Front Motors
vex::motor leftFront(vex::PORT10, ratio6_1, false);
vex::motor rightFront(vex::PORT2, ratio6_1, false);

// Back Motors
vex::motor leftBack(vex::PORT3, ratio6_1, false);
vex::motor rightBack(vex::PORT4, ratio6_1, false);

// Top Motors
vex::motor leftTop(vex::PORT6, ratio6_1, true);
vex::motor rightTop(vex::PORT11, ratio6_1, true);

// Motor Groups
vex::motor_group rightSide(rightFront, rightBack, rightTop);
vex::motor_group leftSide(leftFront, leftBack, leftTop);

// Drivetrain
vex::drivetrain Drivetrain(rightSide, leftSide, 259.34, 320, 40, mm, 1);

// Deadzone
const int DEADZONE_THRESHOLD = 5;

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
  // >>> YOUR MATCH AUTON CODE HERE <<<
  Drivetrain.driveFor(forward, 500, mm);
}

void skillsAuton() {
  // >>> YOUR SKILLS AUTON CODE HERE <<<
  Drivetrain.driveFor(forward, 1500, mm);
}

// --------------------
// PRE-AUTON
// --------------------
void pre_auton(void) {
  // You can show mode selection on the Brain
  Brain1.Screen.clearScreen();
  Brain1.Screen.setCursor(1, 1);
  Brain1.Screen.print("Mode: %s", skillsMode ? "SKILLS" : "MATCH");
}

// --------------------
// AUTONOMOUS
// --------------------
void autonomous(void) {
  if (skillsMode) skillsAuton();
  else matchAuton();
}

// --------------------
// DRIVER CONTROL
// --------------------
void usercontrol(void) {
  while (1) {

    // TOGGLE AUTON MODE USING “LEFT” BUTTON
    if (Controller1.ButtonLeft.pressing()) {
      skillsMode = !skillsMode;
      Brain1.Screen.clearScreen();
      Brain1.Screen.setCursor(1, 1);
      Brain1.Screen.print("Mode: %s", skillsMode ? "SKILLS" : "MATCH");
      wait(300, msec);
    }

    int drivePower = Controller1.Axis3.position(percent);
    int turnPower = Controller1.Axis1.position(percent);

    if (std::abs(drivePower) < DEADZONE_THRESHOLD) drivePower = 0;
    if (std::abs(turnPower) < DEADZONE_THRESHOLD) turnPower = 0;

    int leftOutput = drivePower + turnPower;
    int rightOutput = drivePower - turnPower;

    leftSide.setVelocity(leftOutput, percent);
    rightSide.setVelocity(rightOutput, percent);

    leftSide.spin(forward);
    rightSide.spin(reverse);

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
