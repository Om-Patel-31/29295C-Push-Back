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
  // PID CONTROLLER CLASS
  // --------------------
  // PID (Proportional-Integral-Derivative) controller for precise motor control
  // - Proportional (P): Responds to current error, makes quick corrections
  // - Integral (I): Accumulates error over time, eliminates steady-state errors
  // - Derivative (D): Predicts future error, smooths out overshooting
  // 
  // Usage: Create instance with gains -> calculate() in control loop -> check atTarget()
  // Typical gains: P=0.5-1.0, I=0.01-0.05, D=0.1-0.3 (tune for your robot)
  // 
  class PIDController {
  public:
    double kP, kI, kD;
    double integral = 0.0;      // Accumulated error for I term
    double prevError = 0.0;     // Previous error for D term
    double maxIntegral = 50.0;  // Anti-windup: prevents integral from growing too large
    double tolerance = 1.0;     // How close to target before considered "done" (units depend on usage)
    bool enabled = false;

    // Constructor: Initialize with P, I, D gains
    // Example: PIDController drivePID(0.5, 0.02, 0.1);
    PIDController(double p, double i, double d) : kP(p), kI(i), kD(d) {}

    // Calculate PID output based on error and time delta
    // error: difference between target and current value
    // dt: time since last call in seconds (typically 0.01 for 10ms loop)
    // Returns: control output to apply to motor (-100 to 100 for velocity)
    double calculate(double error, double dt) {
      // Proportional term: immediate response proportional to error
      double pTerm = kP * error;

      // Integral term with anti-windup (prevents accumulation in steady state)
      integral += error * dt;
      if (integral > maxIntegral) integral = maxIntegral;
      if (integral < -maxIntegral) integral = -maxIntegral;
      double iTerm = kI * integral;

      // Derivative term: rate of change prevents overshoot
      double dTerm = 0.0;
      if (dt > 0) {
        dTerm = kD * (error - prevError) / dt;
      }
      prevError = error;

      return pTerm + iTerm + dTerm;
    }

    // Reset integral and error tracking (call before each new movement)
    void reset() {
      integral = 0.0;
      prevError = 0.0;
    }

    // Check if error is within tolerance (movement is complete)
    bool atTarget(double error) {
      return std::abs(error) < tolerance;
    }
  };

  // --------------------
  // ODOMETRY TRACKER
  // --------------------
  // Odometry tracks the robot's position and orientation on the field
  // Updates position using wheel encoder distances and IMU heading
  // Allows for path planning and position-based autonomous routines
  // 
  // Usage: Call update() in control loop -> query position via x/y/heading or use helper functions
  // Note: Accuracy depends on encoder calibration and IMU drift; periodic recalibration recommended
  // 
  class OdometryTracker {
  public:
    double x = 0.0;      // X position (inches) - typically forward/backward
    double y = 0.0;      // Y position (inches) - typically left/right
    double heading = 0.0; // Heading (degrees) - robot orientation, 0-360 or ±180
    double wheelBase = 7.5; // Distance between left/right wheels (inches) - tune for your robot
    double trackingWheelRadius = 1.0; // Tracking wheel radius (inches) - tune for your robot

    // Update position based on encoder distances and IMU heading
    // leftDist: distance traveled by left side (inches)
    // rightDist: distance traveled by right side (inches)
    // imuHeading: current heading from IMU sensor (degrees)
    void update(double leftDist, double rightDist, double imuHeading) {
      // Calculate average distance both sides traveled
      double avgDist = (leftDist + rightDist) / 2.0;

      // Use IMU heading (more accurate than encoder-based heading)
      heading = imuHeading;

      // Update position using forward kinematics
      // Assumes robot moves in direction of heading
      double headingRad = heading * M_PI / 180.0;
      x += avgDist * std::cos(headingRad);
      y += avgDist * std::sin(headingRad);
    }

    // Reset position to origin (0, 0) facing heading 0
    void reset() {
      x = 0.0;
      y = 0.0;
      heading = 0.0;
    }

    // Manually set position and heading (useful for field corrections)
    void setPosition(double newX, double newY, double newHeading) {
      x = newX;
      y = newY;
      heading = newHeading;
    }

    // Calculate straight-line distance from current position to target
    // Useful for determining when robot has reached a location
    double distanceTo(double targetX, double targetY) {
      double dx = targetX - x;
      double dy = targetY - y;
      return std::sqrt(dx * dx + dy * dy);
    }

    // Calculate heading angle needed to face target position
    // Useful for path planning: turn to this heading, then drive
    double headingTo(double targetX, double targetY) {
      double dx = targetX - x;
      double dy = targetY - y;
      return std::atan2(dy, dx) * 180.0 / M_PI;
    }
  };

  // --------------------
  // GLOBALS
  // --------------------
  vex::brain Brain;
  vex::controller Controller1;

  // Front Motors
  vex::motor leftFront(vex::PORT5, ratio6_1, true);
  vex::motor rightFront(vex::PORT10, ratio6_1, false);

  // Back Motors
  vex::motor leftBack(vex::PORT3, ratio6_1, true);
  vex::motor rightBack(vex::PORT7, ratio6_1, false);

  // Top Motors
  vex::motor leftTop(vex::PORT4, ratio6_1, false);
  vex::motor rightTop(vex::PORT8, ratio6_1, true);

  // Motor Groups
  vex::motor_group rightSide(rightFront, rightBack, rightTop);
  vex::motor_group leftSide(leftFront, leftBack, leftTop);

  // Drivetrain
  vex::drivetrain Drivetrain(leftSide, rightSide, 259.34, 320, 40, mm, 1);

  // Motors
  vex::motor intakeMotor(vex::PORT1, ratio6_1, true);
  vex::motor outputMotor(vex::PORT6, ratio6_1, false);

  // Pneumatics (use brain three-wire ports)
  digital_out Blocker(Brain.ThreeWirePort.A);
  digital_out MatchLoader(Brain.ThreeWirePort.B);
  digital_out Pistons(Brain.ThreeWirePort.C);

  // Sensors
  vex::inertial inertialSensor = vex::inertial(PORT21);
  vex::distance DistanceSensor = vex::distance(PORT9);

  // --------------------
  // PID CONTROLLERS & ODOMETRY
  // --------------------
  // PID for driving forward/backward (tune these values for your robot)
  // Start with: kP=0.5, kI=0.02, kD=0.1
  // Increase kP if response is too slow, decrease if overshooting
  // Increase kD if oscillating, decrease if too sluggish
  PIDController drivePID(1.0, 0.01, 0.1);
  
  // PID for turning (tune these values for your robot)
  // Start with: kP=0.8, kI=0.01, kD=0.15
  // Typically needs higher P and D than drive for precise heading control
  PIDController turnPID(0.8, 0.01, 0.15);
  
  // Odometry tracker - maintains robot position and heading throughout match
  OdometryTracker odometry;

  // --------------------
  // CALLBACK HANDLERS
  // --------------------
  // Intake callbacks (R1 = forward, R2 = reverse)
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
  const int DEADZONE_THRESHOLD = 10;

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
  // Reduced to provide smoother acceleration. Tune up if response too sluggish.
  double slewMaxDelta = 7.50;

  // Clamp a value to a min/max range (helper for std::clamp compatibility)
  template<typename T>
  static T clamp(T value, T minVal, T maxVal) {
    if (value > maxVal) return maxVal;
    if (value < minVal) return minVal;
    return value;
  }

  // Clamp a percent value to the valid [-100,100] range
  static double clampPercent(double v) {
    return clamp(v, -100.0, 100.0);
  }

  // Apply an exponential input curve to joystick percent values.
  // exponent >= 1.0. Larger values make low inputs finer and high inputs steeper.
  static double expoCurve(double inputPercent, double exponent) {
    double v = clampPercent(inputPercent);
    double sign = (v >= 0.0) ? 1.0 : -1.0;
    double absNorm = std::abs(v) / 100.0; // 0..1
    double shaped = std::pow(absNorm, exponent) * 100.0;
    return sign * shaped;
  }

  // Global, tunable input exponent (1.0 = linear). Adjust with controller.
  double INPUT_EXPO = 2.0;

  // Increase/decrease expo (bound and show on brain)
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

  // Turn sensitivity (percent). 100 = normal
  int turnSensitivity = 60;
  // Slow-turn sensitivity when Y is held
  const int slowTurnSensitivity = 25;

  // Deceleration step when controller released (percent per loop)
  // Increased to reduce coasting delay when sticks released.

  // --------------------
  // PID-BASED DRIVING FUNCTIONS
  // --------------------
  // Drive forward/backward to a target distance using PID control
  // Hybrid approach: Uses ENCODERS for primary distance measurement + DISTANCE SENSOR for validation/correction
  // This provides more precise and repeatable movements than open-loop driving
  // Parameters:
  //   targetDist: Distance to drive in inches (positive=forward, negative=reverse)
  //   velocity: Max motor velocity in percent (1-100), default 50
  //   timeout: Max time to attempt movement in milliseconds, default 5000
  //   useDistanceSensor: If true, uses distance sensor to validate final position (default true)
  // 
  // How it works:
  // 1. Uses encoders as PRIMARY measurement (works at any distance)
  // 2. When close to target, uses distance sensor to FINE-TUNE final position
  // 3. Distance sensor prevents wheel slip and encoder drift from causing position error
  // 4. Stops when within tolerance or timeout reached
  // 
  void driveWithPID(double targetDist, int velocity = 50, int timeout = 5000, bool useDistanceSensor = true) {
    drivePID.reset();
    drivePID.tolerance = 1.0; // 1 inch tolerance (increased from 0.5)
    
    double leftStartPos = leftSide.position(vex::rotationUnits::rev) * 10.2; // inches (3.25" wheel diameter, circumference ~ 10.2 inches)
    double rightStartPos = rightSide.position(vex::rotationUnits::rev) * 10.2;
    double distanceSensorStartDist = useDistanceSensor ? DistanceSensor.objectDistance(vex::distanceUnits::in) : 0.0;
    
    double elapsedTime = 0;
    bool usingDistanceSensorMode = false;
    
    while (elapsedTime < timeout) {
      double leftCurr = leftSide.position(rev) * 10.2;
      double rightCurr = rightSide.position(rev) * 10.2;
      
      double leftDist = leftCurr - leftStartPos;
      double rightDist = rightCurr - rightStartPos;
      double avgDist = (leftDist + rightDist) / 2.0;
      
      // Primary error calculation from encoders
      double error = targetDist - avgDist;
      
      // If distance sensor available and we're within 6 inches of target, switch to distance sensor for fine-tuning
      if (useDistanceSensor && !usingDistanceSensorMode && std::abs(error) < 6.0) {
        // Switch to distance sensor mode for precision final positioning
        double currentDistSensorReading = DistanceSensor.objectDistance(vex::distanceUnits::in);
        // Calculate target distance sensor reading based on starting distance and target movement
        double targetDistSensorReading = distanceSensorStartDist - targetDist;
        error = targetDistSensorReading - currentDistSensorReading;
        usingDistanceSensorMode = true;
      } else if (useDistanceSensor && usingDistanceSensorMode) {
        // Continue using distance sensor for fine-tuning
        double currentDistSensorReading = DistanceSensor.objectDistance(vex::distanceUnits::in);
        double targetDistSensorReading = distanceSensorStartDist - targetDist;
        error = targetDistSensorReading - currentDistSensorReading;
      }
      
      if (drivePID.atTarget(error)) {
        leftSide.stop(brake);  // Use brake to stop immediately, not coast
        rightSide.stop(brake);
        break;
      }
      
      double pidOutput = drivePID.calculate(error, 0.01); // 10ms control loop
      double driveSpeed = clamp(pidOutput, -100.0, 100.0);
      
      // Prevent reversing - only move forward when error is positive
      if (error <= 0.0) {
        driveSpeed = 0; // Stop if we've reached or passed target
      }
      
      leftSide.setVelocity(std::abs(driveSpeed), percent);
      rightSide.setVelocity(std::abs(driveSpeed), percent);
      
      if (driveSpeed >= 0) {
        leftSide.spin(forward);
        rightSide.spin(forward);
      } else {
        leftSide.spin(reverse);
        rightSide.spin(reverse);
      }
      
      wait(10, msec);
      elapsedTime += 10;
    }
    
    leftSide.stop();
    rightSide.stop();
  }

  // Drive until distance sensor reads a specific distance from an object
  // Useful for precise positioning relative to walls or game objects
  // Parameters:
  //   targetSensorDist: Target distance sensor reading in inches
  //   velocity: Max motor velocity in percent (1-100)
  //   timeout: Max time to attempt movement in milliseconds, default 3000
  //   direction: 1 for forward (toward object), -1 for reverse (away from object)
  // 
  void driveToDistanceSensor(double targetSensorDist, int velocity = 30, int timeout = 3000, int direction = 1) {
    drivePID.reset();
    drivePID.tolerance = 0.3; // Tighter tolerance for distance sensor
    
    double elapsedTime = 0;
    
    while (elapsedTime < timeout) {
      double currentSensorDist = DistanceSensor.objectDistance(vex::distanceUnits::in);
      double error = targetSensorDist - currentSensorDist;
      
      if (drivePID.atTarget(error)) {
        leftSide.stop();
        rightSide.stop();
        break;
      }
      
      double pidOutput = drivePID.calculate(error, 0.01);
      double driveSpeed = clamp(pidOutput, -100.0, 100.0) * direction;
      
      leftSide.setVelocity(std::abs(driveSpeed), percent);
      rightSide.setVelocity(std::abs(driveSpeed), percent);
      
      if (driveSpeed >= 0) {
        leftSide.spin(reverse);
        rightSide.spin(forward);
      } else {
        leftSide.spin(forward);
        rightSide.spin(reverse);
      }
      
      wait(10, msec);
      elapsedTime += 10;
    }
    
    leftSide.stop();
    rightSide.stop();
  }

  // Turn to a target heading using PID control
  // Provides smooth, accurate heading changes with minimal oscillation
  // Parameters:
  //   targetHeading: Desired heading in degrees (can be negative)
  //   timeout: Max time to attempt turn in milliseconds, default 3000
  // 
  // How it works:
  // 1. Reads current heading from IMU
  // 2. Calculates error (target - current), wraps to shortest path
  // 3. Uses PID to calculate ideal turn speed
  // 4. Stops when within tolerance or timeout reached
  // 
  void turnWithPID(double targetHeading, int timeout = 3000) {
    turnPID.reset();
    turnPID.tolerance = 2.0; // 2 degree tolerance
    
    double elapsedTime = 0;
    
    while (elapsedTime < timeout) {
      double currentHeading = inertialSensor.rotation();
      double error = targetHeading - currentHeading;
      
      // Wrap error to shortest path (-180 to 180)
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
        leftSide.spin(reverse);
        rightSide.spin(reverse);
      } else {
        leftSide.spin(forward);
        rightSide.spin(forward);
      }
      
      wait(10, msec);
      elapsedTime += 10;
    }
    
    leftSide.stop();
    rightSide.stop();
  }

  // Update odometry tracker with current encoder and heading data
  // Call this regularly (e.g., every 5-10ms) to keep position estimates accurate
  // Uses encoder positions to calculate distance moved, then updates position based on heading
  void updateOdometry() {
    double currentHeading = inertialSensor.rotation();
    double leftDist = leftSide.position(vex::rotationUnits::rev) * 10.2; // Convert to inches (3.25" wheel)
    double rightDist = rightSide.position(vex::rotationUnits::rev) * 36.0;
    odometry.update(leftDist, rightDist, currentHeading);
  }

  // Reset all motor encoders to zero (call at start of autonomous)
  // Essential for clean distance measurements and preventing accumulated error
  void resetEncoders() {
    leftSide.resetPosition();
    rightSide.resetPosition();
    intakeMotor.resetPosition();
    outputMotor.resetPosition();
  }

  // --------------------
  // MODE SELECTOR
  // --------------------
  bool skillsMode = false;
  bool matchAutonSide = false;  // false = Left, true = Right
  bool autonTestActive = false;
  // false = Match Autonomous
  // true  = Skills Autonomous

  // --------------------
  // AUTON FUNCTIONS
  // --------------------
  // Forward declaration for skills autonomous (defined below)
  void skillsAuton();

  void matchAutonRight() {
    // Right side match autonomous routine
    Brain.Screen.clearLine(3);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Match Auton: RIGHT");
    
    // Reset encoders and odometry
    resetEncoders();
    odometry.reset();
    drivePID.reset();
    turnPID.reset();
    
    // 1) Drive forward 5 inches with PID
    driveWithPID(5, 50, 5000, true);
    wait(1, msec);
    updateOdometry();

    Controller1.rumble(".-");
  }

  void matchAutonLeft() {
    // Left side match autonomous routine
    Brain.Screen.clearLine(3);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Match Auton: LEFT");
    
    // Reset encoders and odometry
    resetEncoders();
    odometry.reset();
    drivePID.reset();
    turnPID.reset();
    
    // 1) Drive forward 5 inches with PID
    driveWithPID(50, 50, 5000, true);
    wait(1, msec);
    updateOdometry();

    Controller1.rumble(".-");
  }

  void skillsAuton() {
    // Signal start
    intakeMotor.spin(reverse);
    Controller1.Screen.clearLine(1);
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Starting Skills Auton");
    
    // Reset encoders for clean distance measurements
    resetEncoders();
    
    // Reset odometry at start
    odometry.reset();
    drivePID.reset();
    turnPID.reset();
    
    Drivetrain.setDriveVelocity(50, percent);
    Drivetrain.setTurnVelocity(25, percent);
    
    // 1) Drive forward 25 inches using PID (hybrid: encoder + distance sensor for fine-tune)
    driveWithPID(25, 50, 5000, true);
    wait(100, msec);
    updateOdometry();

    // 2) Toggle match loader (extend)
    matchLoaderToggle();
    wait(100, msec);

    // 3) Spin intake in reverse for 2500 ms
    wait(2500, msec);
    intakeMotor.stop();

    // 4) Toggle match loader (retract)
    matchLoaderToggle();
    wait(100, msec);

    // 5) Turn -30 degrees using PID
    turnWithPID(-30, 3000);
    wait(100, msec);
    updateOdometry();
    
    // 6) Move forward 17 inches using PID (hybrid approach)
    driveWithPID(17, 50, 4000, true);
    wait(100, msec);
    updateOdometry();
    
    // 7) Spin intake forward for 1500 ms
    intakeMotor.spin(forward);
    outputMotor.spin(reverse);
    wait(1500, msec);
    intakeMotor.stop();
    
    // Signal end
    
    // 1) Drive reverse 20 inches using PID (hybrid)
    driveWithPID(-20, 50, 5000, true);
    wait(100, msec);
    updateOdometry();
    
    // 2) Turn left to -75 degrees using PID
    turnWithPID(-75, 3000);
    wait(100, msec);
    updateOdometry();
    
    // 3) Drive forward 50 inches using PID (hybrid)
    driveWithPID(50, 50, 6000, true);
    wait(100, msec);
    updateOdometry();
    
    // 4) Toggle match loader
    matchLoaderToggle();
    wait(100, msec);
    
    // 5) Spin intake reverse for 1500 ms
    intakeMotor.spin(reverse);
    wait(1500, msec);
    intakeMotor.stop();
    
    // 6) Toggle match loader
    matchLoaderToggle();
    wait(100, msec);
    
    // 8) Turn right to -25 degrees using PID
    turnWithPID(-25, 3000);
    wait(100, msec);
    updateOdometry();
    
    // 9) Drive forward 7 inches using PID (hybrid) - fine positioning
    driveWithPID(7, 50, 3000, true);
    wait(100, msec);
    updateOdometry();
    
    // 10) Toggle blocker
    blockerToggle();
    wait(100, msec);
    
    // 11) Run both intake and output for 2500 ms
    intakeMotor.spin(forward);
    outputMotor.spin(forward);
    wait(2500, msec);
    intakeMotor.stop();
    outputMotor.stop();
    
    // 12) Drive reverse 20 inches using PID (hybrid)
    driveWithPID(-20, 50, 5000, true);
    wait(100, msec);
    updateOdometry();
    
    // 13) Turn left 60 degrees (heading = -85)
    turnWithPID(-85, 3000);
    wait(100, msec);
    updateOdometry();
    
    // 14) Drive reverse 10 inches using PID (hybrid)
    driveWithPID(-10, 50, 3500, true);
    wait(100, msec);
    updateOdometry();

    // 15) Turn left 90 degrees (heading = -175)
    turnWithPID(-175, 3000);
    wait(100, msec);
    updateOdometry();
    
    // 16) Drive reverse 5 inches using PID (hybrid)
    driveWithPID(-5, 50, 2500, true);
    wait(100, msec);
    updateOdometry();

    // Stop all motors before ending autonomous
    intakeMotor.stop();
    outputMotor.stop();
    leftSide.stop();
    rightSide.stop();

    Controller1.rumble(".-");
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
    // Show mode selection on the Brain
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Mode: %s", skillsMode ? "SKILLS" : "MATCH");

    // Set default stopping modes and bind controller callbacks
    // Use coast so robot rolls down naturally in deadzone
    leftSide.setStopping(vex::brakeType::coast);
    rightSide.setStopping(vex::brakeType::coast);
    intakeMotor.setStopping(vex::brakeType::brake);
    outputMotor.setStopping(vex::brakeType::brake);

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
    Controller1.ButtonUp.pressed(increaseExpo);
    Controller1.ButtonDown.pressed(decreaseExpo);
  }

  // --------------------
  // AUTONOMOUS
  // --------------------
  void autonomous(void) {
    // Ensure IMU is calibrated before autonomous
    calibrateIMU();
    if (skillsMode) skillsAuton();
    else if (matchAutonSide) matchAutonRight();
    else matchAutonLeft();
  }

  // --------------------
  // DRIVER CONTROL
  // --------------------
  void usercontrol(void) {

    calibrateIMU();
    odometry.reset(); // Reset odometry at start of driver control

    double currentLeft = 0.0;
    double currentRight = 0.0;

    
    while (1) {
      Controller1.Screen.print(DistanceSensor.objectDistance(inches));
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

      // Scale turning sensitivity (use Y to enable slow-turn)
      int activeTurnSens = Controller1.ButtonY.pressing() ? slowTurnSensitivity : turnSensitivity;
      int scaledTurn = (turnPower * activeTurnSens) / 100;

      int leftOutput = drivePower + scaledTurn;
      int rightOutput = drivePower - scaledTurn;

      // Apply exponential input curve for smoother low-end response
      double targetLeft = expoCurve((double)leftOutput, INPUT_EXPO);
      double targetRight = expoCurve((double)rightOutput, INPUT_EXPO);

      // Apply slew rate limiting for smooth acceleration/deceleration
      currentLeft = slewTo(targetLeft, currentLeft, slewMaxDelta);
      currentRight = slewTo(targetRight, currentRight, slewMaxDelta);

      // Set velocities with slewed values
      leftSide.setVelocity(std::abs(currentLeft), percent);
      rightSide.setVelocity(std::abs(currentRight), percent);

      // Only spin if not zero, otherwise coast
      if (std::abs(currentLeft) < 0.5 && std::abs(currentRight) < 0.5) {
        leftSide.stop(coast);
        rightSide.stop(coast);
      } else {
        leftSide.spin(currentLeft >= 0.0 ? forward : reverse);
        rightSide.spin(currentRight >= 0.0 ? forward : reverse);
      }

      // Update odometry
      updateOdometry();

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

      // Display odometry information
      Brain.Screen.clearLine(7);
      Brain.Screen.setCursor(7, 1);
      Brain.Screen.print("X:%.1f Y:%.1f HDG:%.1f", odometry.x, odometry.y, odometry.heading);

      wait(0, msec);
    }

    }

  // --------------------
  // MAIN
  // --------------------
  int main() {
    // Run pre-auton initialization first (calibration, bindings)
    pre_auton();

    // Register competition callbacks
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
  }
