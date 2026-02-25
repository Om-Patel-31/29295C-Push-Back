#ifndef DRIVERCONTROL_H
#define DRIVERCONTROL_H

#include "vex.h"
#include <cmath>

// Driver input handling and shaping for differential-drive robots.
//
// Keep comments short: modes describe how controller axes map to
// drivetrain commands. Inputs are read in percent (-100..100). Sensitivities
// let you tune responsiveness without changing motor code.

// Control scheme types
enum class ControlType {
  ARCADE,           // Left stick Y = drive, Right stick X = turn
  TANK,             // Left stick Y = left side, Right stick Y = right side
  SPLIT_ARCADE,     // Left stick Y = drive, Left stick X = turn
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
  
  // Input filtering (smooths joystick noise)
  double filteredDrive;
  double filteredTurn;
  double filteredLeft;
  double filteredRight;
  double filterAlpha; // Low-pass filter coefficient (0.1 = heavy smoothing, 0.5 = light)
  
  // Apply smooth deadzone with soft transition
  double applyDeadzoneSoft(int value) {
    double v = std::abs(value);
    if (v < deadzone) return 0.0;
    // Smooth cubic transition just above deadzone
    if (v < deadzone + 10) {
      double t = (v - deadzone) / 10.0;
      return (value >= 0 ? 1.0 : -1.0) * t * t * (3.0 - 2.0 * t) * (v - deadzone);
    }
    return (double)value;
  }
  
  // Legacy deadzone for backwards compatibility
  int applyDeadzone(int value) {
    return (std::abs(value) < deadzone) ? 0 : value;
  }
  
  // Apply direct linear scaling to joystick input.
  double applyLinear(double inputPercent) {
    return std::fmin(std::fmax(inputPercent, -100.0), 100.0);
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
      slowTurnSensitivity(slowTurnSens),
      filteredDrive(0.0),
      filteredTurn(0.0),
      filteredLeft(0.0),
      filteredRight(0.0),
      filterAlpha(0.3) {} // 0.3 = smooth but responsive (like PROS)

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
  
  // Low-pass filter for smooth joystick input
  // Removes jitter and high-frequency noise while preserving responsiveness
  double lowPassFilter(double& filtered, double raw) {
    filtered = filtered * (1.0 - filterAlpha) + raw * filterAlpha;
    return filtered;
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
        drivePower = applyDeadzone((int)lowPassFilter(filteredDrive, getAxisPosition(controller, driveAxis)));
        turnPower = applyDeadzone((int)lowPassFilter(filteredTurn, getAxisPosition(controller, turnAxis)));
        break;
        
      case ControlType::TANK:
        // Left stick Y = left side, Right stick Y = right side
        {
          int leftPower = applyDeadzone((int)lowPassFilter(filteredLeft, getAxisPosition(controller, leftAxis)));
          int rightPower = applyDeadzone((int)lowPassFilter(filteredRight, getAxisPosition(controller, rightAxis)));
          leftOutput = applyLinear(leftPower);
          rightOutput = applyLinear(rightPower);
          return; // Tank doesn't use drive/turn combination
        }
        
      case ControlType::SPLIT_ARCADE:
        // Left stick Y = drive, Left stick X = turn
        drivePower = applyDeadzone((int)lowPassFilter(filteredDrive, getAxisPosition(controller, driveAxis)));
        turnPower = applyDeadzone((int)lowPassFilter(filteredTurn, getAxisPosition(controller, turnAxis)));
        break;
        
      case ControlType::SINGLE_STICK:
        // Right stick only: Y = drive, X = turn
        drivePower = applyDeadzone((int)lowPassFilter(filteredDrive, getAxisPosition(controller, driveAxis)));
        turnPower = applyDeadzone((int)lowPassFilter(filteredTurn, getAxisPosition(controller, turnAxis)));
        break;
    }
    
    // Apply turn sensitivity
    int activeTurnSens = slowTurn ? slowTurnSensitivity : turnSensitivity;
    int scaledTurn = (turnPower * activeTurnSens) / 100;
    
    // Apply linear input mapping
    double targetDrive = applyLinear(drivePower);
    double targetTurn = applyLinear(scaledTurn);
    
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
  
  // Filter control for input smoothing (0.1 = heavy smoothing, 0.5 = light)
  void setFilterAlpha(double alpha) { 
    filterAlpha = std::fmin(std::fmax(alpha, 0.1), 1.0); 
  }
  double getFilterAlpha() const { return filterAlpha; }
  
  // Reset to default sensitivity settings
  void resetDefaults() {
    exponent = 2.0;
    turnSensitivity = 60;
    slowTurnSensitivity = 50;
    filterAlpha = 0.3;
  }
};

#endif // DRIVERCONTROL_H