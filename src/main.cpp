/*----------------------------------------------------------------------------*/
/* */
/* Module: main.cpp */
/* Author: */
/* Created: 10/9/2025, 12:37:12 PM */
/* Description: V5 project */
/* */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath> // Required for std::abs()
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
vex::brain Brain1;
vex::controller Controller1;

// Front Motors
vex::motor leftFront(vex::PORT1, ratio6_1);
vex::motor rightFront(vex::PORT2, ratio6_1);
// Back Motors
vex::motor leftBack(vex::PORT3, ratio6_1);
vex::motor rightBack(vex::PORT4, ratio6_1);
// Top Motors
vex::motor leftBack(vex::PORT5, ratio6_1, true);
vex::motor rightTop(vex::PORT6, ratio6_1, true);

// Motor Groups
vex::motor_group rightSide(rightFront, rightBack, rightTop);
vex::motor_group leftSide(leftFront, leftBack, leftTop);

vex::drivetrain Drivetrain(rightSide, leftSide, 259.34, 320, 40, mm, 1);

// Deadzone threshold constant
const int DEADZONE_THRESHOLD = 5; // Adjust this value (5-10 is typical)

/*---------------------------------------------------------------------------*/
/* Pre-Autonomous Functions */
/* */
/* You may want to perform some actions before the competition starts. */
/* Do them in the following function. You must return from this function */
/* or the autonomous and usercontrol tasks will not be started. This */
/* function is only called once after the V5 has been powered on and */
/* not every time that the robot is disabled. */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

}

/*---------------------------------------------------------------------------*/
/* */
/* Autonomous Task */
/* */
/* This task is used to control your robot during the autonomous phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/* */
/* User Control Task */
/* */
/* This task is used to control your robot during the user control phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // Get the position of the joysticks
    int drivePower = Controller1.Axis3.position(percent); // Left joystick Y-axis (Forward/Reverse)
    int turnPower = Controller1.Axis1.position(percent); // Right joystick X-axis (Turning)

    // Apply Deadzone Logic:
    // If the absolute value of the joystick input is less than the threshold, set it to 0.
    if (std::abs(drivePower) < DEADZONE_THRESHOLD) {
      drivePower = 0;
    }
    if (std::abs(turnPower) < DEADZONE_THRESHOLD) {
      turnPower = 0;
    }

    // Calculate motor velocities for arcade drive
    // Left side velocity is forward power + turn power
    // Right side velocity is forward power - turn power
    int leftVelocity = drivePower + turnPower;
    int rightVelocity = drivePower - turnPower;
    
    leftSide.setVelocity(leftVelocity, percent);
    rightSide.setVelocity(rightVelocity, percent);

    // Spin the motors continuously at the set velocity
    leftSide.spin(forward);
    rightSide.spin(forward);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
