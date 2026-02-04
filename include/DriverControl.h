#ifndef DRIVERCONTROL_H
#define DRIVERCONTROL_H

#include "vex.h"
#include <cmath>

// Driver input handling and shaping for differential-drive robots.
//
// Keep comments short: modes describe how controller axes map to
// drivetrain commands. Inputs are read in percent (-100..100). Curves
// and sensitivities let you tune responsiveness without changing motor code.

// Control scheme types
enum class ControlType {
  ARCADE,           // Left stick Y = drive, Right stick X = turn
  TANK,             // Left stick Y = left side, Right stick Y = right side
  SPLIT_ARCADE,     // Left stick Y = drive, Left stick X = turn
  CURVATURE,        // Arcade with velocity-based turn scaling
  SINGLE_STICK      // Single stick: Y = drive, X = turn
};

class DriverControl {
private:
  ControlType controlType;
  int deadzone;
  double exponent;
  int turnSensitivity;
  int slowTurnSensitivity;
  // Axis mapping (1-4 correspond to Controller.Axis1..Axis4)
  int driveAxis;
  int turnAxis;
  int leftAxis;
  int rightAxis;
  
  // Apply deadzone to joystick input
  int applyDeadzone(int value) {
    return (std::abs(value) < deadzone) ? 0 : value;
  }
  
  // Apply exponential curve to input
  // Applies an exponential shaping curve to joystick percent input.
  // - `inputPercent` expected in [-100,100].
  // - `expo` >1 makes low inputs finer and high inputs stronger.
  double applyCurve(double inputPercent, double expo) {
    double v = std::fmin(std::fmax(inputPercent, -100.0), 100.0);
    double sign = (v >= 0.0) ? 1.0 : -1.0;
    double absNorm = std::abs(v) / 100.0;
    double shaped = std::pow(absNorm, expo) * 100.0;
    return sign * shaped;
  }
  
  // Clamp value to -100 to 100
  double clamp(double value) {
    return std::fmin(std::fmax(value, -100.0), 100.0);
  }

public:
  DriverControl(ControlType type = ControlType::ARCADE, 
                int deadzoneThreshold = 10,
                double inputExponent = 2.0,
                int turnSens = 60,
                int slowTurnSens = 50)
    : controlType(type),
      deadzone(deadzoneThreshold),
      exponent(inputExponent),
      turnSensitivity(turnSens),
      slowTurnSensitivity(slowTurnSens) {}

  // Initialize default axis mapping based on control type
  void initDefaultAxes() {
    // Default mappings matching previous behavior
    driveAxis = 3;
    turnAxis = 1;
    leftAxis = 3;
    rightAxis = 2;

    if (controlType == ControlType::SPLIT_ARCADE) {
      driveAxis = 3;
      turnAxis = 4;
    } else if (controlType == ControlType::SINGLE_STICK) {
      driveAxis = 2;
      turnAxis = 1;
    } else if (controlType == ControlType::TANK) {
      leftAxis = 3;
      rightAxis = 2;
    }
  }

  // Helper to read a controller axis by numeric id (1..4)
  int getAxisPosition(vex::controller& controller, int axisNumber) {
    switch (axisNumber) {
      case 1: return controller.Axis1.position(vex::percent);
      case 2: return controller.Axis2.position(vex::percent);
      case 3: return controller.Axis3.position(vex::percent);
      case 4: return controller.Axis4.position(vex::percent);
      default: return 0;
    }
  }
  
  // Calculate motor outputs based on controller input
  // `calculate` reads the selected axes, applies deadzone and shaping,
  // scales turning by sensitivity, and writes final left/right outputs
  // in percent (-100..100). For TANK mode it returns direct sides.
  void calculate(vex::controller& controller, double& leftOutput, double& rightOutput, bool slowTurn = false) {
    int drivePower = 0;
    int turnPower = 0;
    // ensure axis mapping is initialized
    initDefaultAxes();
    
    switch (controlType) {
      case ControlType::ARCADE:
        // Left stick Y = drive, Right stick X = turn
        drivePower = applyDeadzone(getAxisPosition(controller, driveAxis));
        turnPower = applyDeadzone(getAxisPosition(controller, turnAxis));
        break;
        
      case ControlType::TANK:
        // Left stick Y = left side, Right stick Y = right side
        {
          int leftPower = applyDeadzone(getAxisPosition(controller, leftAxis));
          int rightPower = applyDeadzone(getAxisPosition(controller, rightAxis));
          leftOutput = applyCurve(leftPower, exponent);
          rightOutput = applyCurve(rightPower, exponent);
          return; // Tank doesn't use drive/turn combination
        }
        
      case ControlType::SPLIT_ARCADE:
        // Left stick Y = drive, Left stick X = turn
        drivePower = applyDeadzone(getAxisPosition(controller, driveAxis));
        turnPower = applyDeadzone(getAxisPosition(controller, turnAxis));
        break;
        
      case ControlType::CURVATURE:
        // Arcade with velocity-based turn scaling
        drivePower = applyDeadzone(getAxisPosition(controller, driveAxis));
        turnPower = applyDeadzone(getAxisPosition(controller, turnAxis));
        // Scale turn based on velocity for better control at low speeds
        if (std::abs(drivePower) > 5) {
          // reduce turn when driving fast: simple linear scale factor
          turnPower = (int)(turnPower * (1.0 - std::abs(drivePower) / 200.0));
        }
        break;
        
      case ControlType::SINGLE_STICK:
        // Right stick only: Y = drive, X = turn
        drivePower = applyDeadzone(getAxisPosition(controller, driveAxis));
        turnPower = applyDeadzone(getAxisPosition(controller, turnAxis));
        break;
    }
    
    // Apply turn sensitivity
    int activeTurnSens = slowTurn ? slowTurnSensitivity : turnSensitivity;
    int scaledTurn = (turnPower * activeTurnSens) / 100;
    
    // Apply expo curve
    double targetDrive = applyCurve(drivePower, exponent);
    double targetTurn = applyCurve(scaledTurn, exponent);
    
    // Calculate final outputs
    leftOutput = clamp(targetDrive + targetTurn);
    rightOutput = clamp(targetDrive - targetTurn);
  }
  
  // Setters for adjusting parameters on the fly
  void setControlType(ControlType type) { controlType = type; }
  void setDeadzone(int value) { deadzone = value; }
  void setExponent(double value) { exponent = value; }
  void setTurnSensitivity(int value) { turnSensitivity = value; }
  void setSlowTurnSensitivity(int value) { slowTurnSensitivity = value; }
  // Axis setters/getters
  void setDriveAxis(int axis) { driveAxis = axis; }
  void setTurnAxis(int axis) { turnAxis = axis; }
  void setLeftAxis(int axis) { leftAxis = axis; }
  void setRightAxis(int axis) { rightAxis = axis; }
  int getDriveAxis() const { return driveAxis; }
  int getTurnAxis() const { return turnAxis; }
  int getLeftAxis() const { return leftAxis; }
  int getRightAxis() const { return rightAxis; }
  
  // Getters
  ControlType getControlType() const { return controlType; }
  int getDeadzone() const { return deadzone; }
  double getExponent() const { return exponent; }
  int getTurnSensitivity() const { return turnSensitivity; }
  int getSlowTurnSensitivity() const { return slowTurnSensitivity; }
};

#endif // DRIVERCONTROL_H