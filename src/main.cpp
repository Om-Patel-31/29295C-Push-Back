#include "DriverControl.h"
#include "OdometryTracker.h"
#include "PIDController.h"
#include "vex.h"
#include <cmath>

using namespace	vex;

competition		Competition;

// DRIVER CONTROL CONFIGURATION - Change this to switch control schemes
DriverControl	driverControl(ControlType::ARCADE, 10, 2.0, 60, 50);

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
vex::motor_group intakeGroup(intakeMotor, outputMotor);

digital_out		Blocker(Brain.ThreeWirePort.A);
digital_out		MatchLoader(Brain.ThreeWirePort.B);
digital_out		Pistons(Brain.ThreeWirePort.C);

vex::inertial inertialSensor = vex::inertial(PORT21);

PIDController	drivePID(5, 0.5, 0.01);
PIDController	turnPID(0.5, 0.01, 0.05);
OdometryTracker	odometry;

// Forward declarations for functions defined later
void			updateOdometry(void);
void			calibrateIMU(void);
extern bool		matchAutonSide;

const int		IMU_distance = 13;

void	intakeForwardPressed(void)
{
	intakeMotor.spin(forward);
}
void	intakeForwardReleased(void)
{
	if (!Controller1.ButtonR2.pressing())
	{
		intakeMotor.stop();
	}
}
void	intakeReversePressed(void)
{
	intakeMotor.spin(reverse);
}
void	intakeReverseReleased(void)
{
	if (!Controller1.ButtonR1.pressing())
		intakeMotor.stop();
}

void	outputForwardPressed(void)
{
	outputMotor.spin(forward);
}
void	outputForwardReleased(void)
{
	if (!Controller1.ButtonL2.pressing())
	{
		outputMotor.stop();
	}
}
void	outputReversePressed(void)
{
	outputMotor.spin(reverse);
}
void	outputReverseReleased(void)
{
	if (!Controller1.ButtonL1.pressing())
	{
		outputMotor.stop();
	}
}

void	SpinBothForwardPressed(void)
{
	intakeForwardPressed();
	outputForwardPressed();
}

void	SpinBothForwardReleased(void)
{
	intakeForwardReleased();
	outputForwardReleased();
}

void	SpinBothReversePressed(void)
{
	intakeReversePressed();
	outputReversePressed();
}

void	SpinBothReverseReleased(void)
{
	intakeReverseReleased();
	outputReverseReleased();
}

bool			blockerExtended = false;
bool			matchLoaderExtended = false;
bool			pistonsExtended = false;

void	blockerToggle(void)
{
	blockerExtended = !blockerExtended;
	Blocker.set(blockerExtended);
	Brain.Screen.clearLine(2);
	Brain.Screen.setCursor(2, 1);
	Brain.Screen.print("Blocker: %s", blockerExtended ? "EXT" : "CL");
}

void	matchLoaderToggle(void)
{
	matchLoaderExtended = !matchLoaderExtended;
	MatchLoader.set(matchLoaderExtended);
	Brain.Screen.clearLine(4);
	Brain.Screen.setCursor(4, 1);
	Brain.Screen.print("MatchLD: %s", matchLoaderExtended ? "EXT" : "CL");
}

void	pistonsToggle(void)
{
	pistonsExtended = !pistonsExtended;
	Pistons.set(pistonsExtended);
	Brain.Screen.clearLine(5);
	Brain.Screen.setCursor(5, 1);
	Brain.Screen.print("Pistons: %s", pistonsExtended ? "EXT" : "CL");
}

// Auton side selector handlers (Up = RIGHT, Down = LEFT)
void	selectRightAuton(void)
{
	matchAutonSide = true;
	Brain.Screen.clearLine(2);
	Brain.Screen.setCursor(2, 1);
	Brain.Screen.print("Auton: RIGHT");
}

void	selectLeftAuton(void)
{
	matchAutonSide = false;
	Brain.Screen.clearLine(2);
	Brain.Screen.setCursor(2, 1);
	Brain.Screen.print("Auton: LEFT");
}

template <typename T> static T clamp(T value, T minVal, T maxVal)
{
	if (value > maxVal)
		return (maxVal);
	if (value < minVal)
		return (minVal);
	return (value);
}

void	increaseExpo(void)
{
	double	currentExpo;

	currentExpo = driverControl.getExponent();
	if (currentExpo < 5.0)
	{
		driverControl.setExponent(std::round((currentExpo + 0.1) * 100.0)
			/ 100.0);
		Controller1.Screen.clearLine(3);
		Controller1.Screen.setCursor(3, 1);
		Controller1.Screen.print("Expo: %.2f", driverControl.getExponent());
	}
}

void	decreaseExpo(void)
{
	double	currentExpo;

	currentExpo = driverControl.getExponent();
	if (currentExpo > 1.0)
	{
		driverControl.setExponent(std::round((currentExpo - 0.1) * 100.0)
			/ 100.0);
		Controller1.Screen.clearLine(3);
		Controller1.Screen.setCursor(3, 1);
		Controller1.Screen.print("Expo: %.2f", driverControl.getExponent());
	}
}

void	increaseTurnSens(void)
{
	int	currentSens;

	currentSens = driverControl.getTurnSensitivity();
	if (currentSens < 100)
	{
		driverControl.setTurnSensitivity(currentSens + 5);
		Brain.Screen.clearLine(3);
		Brain.Screen.setCursor(3, 1);
		Brain.Screen.print("Turn Sens: %d", driverControl.getTurnSensitivity());
	}
}

void	decreaseTurnSens(void)
{
	int	currentSens;

	currentSens = driverControl.getTurnSensitivity();
	if (currentSens > 10)
	{
		driverControl.setTurnSensitivity(currentSens - 5);
		Brain.Screen.clearLine(3);
		Brain.Screen.setCursor(3, 1);
		Brain.Screen.print("Turn Sens: %d", driverControl.getTurnSensitivity());
	}
}

void	driveWithPID(double targetDist, int velocity = 100, int timeout = 5000,
		bool useDistanceSensor = false)
{
	double leftStartPos;
	double rightStartPos;
	double elapsedTime;
	double leftCurr;
	double rightCurr;
	double leftDist;
	double rightDist;
	double avgDist;
	double error;
	double pidOutput;
	double driveSpeed;

	drivePID.reset();
	drivePID.tolerance = 2.0;
	startX = odometry.x;
	startY = odometry.y;
	startHeading = odometry.heading;
	headingPrevError = 0.0;
	headingKp = 0.40;
	headingKd = 0.05;
	headingKi = 0.03;
	headingIntegral = 0.0;
	headingIntegralLimit = 15.0;
	currentDriveSpeed = 0.0;
	maxDeltaPerSec = 80.0;
	slowZone = 10.0;
	decelK = 0.05;
	minDecelScale = 0.45;
	minApproachSpeed = 6.0;
	headingErrorFiltered = 0.0;
	headingDeadband = 0.5;
	headingFilterAlpha = 0.2;
	elapsedTimeMs = 0;
	while (elapsedTimeMs < timeout)
	{
		// Update odometry first to get accurate position
		updateOdometry();

		// Calculate signed distance along the initial heading direction
		dx = odometry.x - startX;
		dy = odometry.y - startY;
		headingRad = startHeading * M_PI / 180.0;
		currentDist = dx * std::cos(headingRad) + dy * std::sin(headingRad);

		error = targetDist - currentDist;
		if (drivePID.atTarget(error))
		{
			leftSide.stop(brake);
			rightSide.stop(brake);
			break ;
		}
		pidOutput = drivePID.calculate(error, kLoopDt);
		driveSpeed = clamp(pidOutput, -static_cast<double>(velocity),
				static_cast<double>(velocity));
		leftSide.setVelocity(std::abs(driveSpeed), percent);
		rightSide.setVelocity(std::abs(driveSpeed), percent);
		if (driveSpeed >= 0)
		{
			leftSide.spin(forward);
			else leftSide.spin(reverse);

			if (rightSpeed >= 0)
				rightSide.spin(forward);
			else
				rightSide.spin(reverse);

			wait(kLoopMs, msec);
			elapsedTimeMs += kLoopMs;
		}
		leftSide.stop();
		rightSide.stop();
	}

	void turnWithPID(double targetHeading, int timeout = 3000)
	{
		double elapsedTime;
		double currentHeading;
		double error;
		double pidOutput;
		double turnSpeed;

		turnPID.reset();
		turnPID.tolerance = 1.0;
		currentTurnSpeed = 0.0;
		maxTurnDeltaPerSec = 80.0;
		turnSlowZone = 15.0;
		turnDecelK = 0.05;
		minTurnSpeed = 6.0;
		elapsedTime = 0;
		while (elapsedTime < timeout)
		{
			currentHeading = inertialSensor.rotation();
			error = targetHeading - currentHeading;
			if (error > 180)
				error -= 360;
			if (error < -180)
				error += 360;
			if (turnPID.atTarget(error))
			{
				leftSide.stop();
				rightSide.stop();
				break ;
			}
			pidOutput = turnPID.calculate(error, kLoopDt);
			requestedTurnSpeed = clamp(pidOutput, -100.0, 100.0);
			// Smooth decel near target for gentle turn exit
			if (std::abs(error) < turnSlowZone)
			{
				double decelScale;

				decelScale = std::exp(-0.10 * (turnSlowZone - std::abs(error)));
				decelScale = std::fmax(0.90, decelScale);
				requestedTurnSpeed *= decelScale;
			}
			// Smooth ramp for turn transitions
			{
				double maxDelta;

				maxDelta = maxTurnDeltaPerSec * kLoopDt;
				if (requestedTurnSpeed - currentTurnSpeed > maxDelta)
					currentTurnSpeed += maxDelta;
				else if (currentTurnSpeed - requestedTurnSpeed > maxDelta)
					currentTurnSpeed -= maxDelta;
				else
					currentTurnSpeed = requestedTurnSpeed;
			}
			turnSpeed = currentTurnSpeed;
			leftSide.setVelocity(std::abs(turnSpeed), percent);
			rightSide.setVelocity(std::abs(turnSpeed), percent);
			if (turnSpeed >= 0)
			{
				leftSide.spin(forward);
				rightSide.spin(reverse);
			}
			else
			{
				leftSide.spin(reverse);
				rightSide.spin(forward);
			}
			// Update odometry at 1 ms intervals during auton turns
			updateOdometry();
			wait(kLoopMs, msec);
			elapsedTime += kLoopMs;
		}
		leftSide.stop();
		rightSide.stop();
	}

	void updateOdometry(void)
	{
		double currentHeading;
		double leftDist;
		double rightDist;

		currentHeading = inertialSensor.rotation();
		currentLeftPos = leftSide.position(vex::rotationUnits::rev) * 10.2;
		currentRightPos = rightSide.position(vex::rotationUnits::rev) * 10.2;
		leftDelta = currentLeftPos - prevLeftPos;
		rightDelta = currentRightPos - prevRightPos;

		// Only suppress distance if wheels are moving in opposite directions (pure rotation)
		// If motors move in the same direction,
			report the distance (even if curved)
		if (leftDelta * rightDelta < 0) // Opposite signs = opposite directions
		{
			// Pure rotation: suppress distance update
			odometry.update(0.0, 0.0, currentHeading);
		}
		else
		{
			// Same direction or one wheel stopped: report the movement
			odometry.update(leftDelta, rightDelta, currentHeading);
		}

		prevLeftPos = currentLeftPos;
		prevRightPos = currentRightPos;
	}

	void resetEncoders(void)
	{
		leftSide.resetPosition();
		rightSide.resetPosition();
		intakeGroup.resetPosition();
	}

	bool skillsMode = false;
	bool matchAutonSide = true;
	bool autonTestActive = false;

	// --------------------
	// AUTON FUNCTIONS
	// --------------------
	// Forward declaration for skills autonomous
	void skillsAuton(void);

	void matchAutonRight(void)
	{
		calibrateIMU();
		Brain.Screen.clearLine(3);
		Brain.Screen.setCursor(3, 1);
		Brain.Screen.print("Match Auton: RIGHT");
		resetEncoders();
		odometry.reset();
		drivePID.reset();
		turnPID.reset();
		matchLoaderToggle();
		intakeGroup.spin(forward);
		driveWithPID(std::sqrt(2275.0) - IMU_distance);
		updateOdometry();
		turnWithPID(20);
		driveWithPID(5);
		intakeGroup.spin(forward);
		matchLoaderToggle();
		driveWithPID(5);
		intakeGroup.spin(forward);
		wait(4000, msec);
		matchLoaderToggle();
		driveWithPID(8);
		turnWithPID(-40);
		driveWithPID(14);
		intakeGroup.spin(reverse);
		wait(5000, msec);
		Controller1.rumble(".-");
	}

	void matchAutonRightAveryVersion(void)
	{
		calibrateIMU();
		resetEncoders();

		Brain.Screen.print("Avery Match Auton: RIGHT");

		odometry.reset();
		drivePID.reset();
		turnPID.reset();

		// driveWithPID(10); drives forward
		// turnWithPID(90); turns right

		// MATCH LOADER + LONG GOAL SCORING PSEUDOCODE
		// drive forward
		// right 90 deg
		// drive forward
		// right 90 deg
		// drive forward + match load + intake run
		// reverse + line up with long goal
		// release balls into long goal

		driveWithPID(22);
		turnWithPID(90);
		updateOdometry();

		driveWithPID(42);
		turnWithPID(180);
		updateOdometry();

		driveWithPID(14);
		intakeMotor.spin(forward);
		wait(3000, msec);
		intakeMotor.stop();
		turnWithPID(180);
		driveWithPID(-40);
		updateOdometry();

		intakeMotor.spin(reverse);
		outputMotor.spin(reverse);
		wait(4000, msec);
		intakeMotor.stop();
		outputMotor.stop();
		updateOdometry();

		// CENTRE SCORING AND MATCH LOADER + LONG GOAL SCORING PSEUDOCODE
		// drive forward
		// turn right slightly
		// drive forward
		// intake
		// turn left to face centre goal
		// drive forward
		// score in centre goal by reverse intaking
		// drive backward
		// spin 180deg
		// drive forward
		// turn right slightly
		// drive forward
		// intake from match loader
		// drive backward
		// spin 180
		// drive forward
		// intake to put balls in long goal
	}

	void matchAutonLeft(void)
	{
		Brain.Screen.clearLine(3);
		Brain.Screen.setCursor(3, 1);
		Brain.Screen.print("Match Auton: LEFT");
		resetEncoders();
		odometry.reset();
		drivePID.reset();
		turnPID.reset();
		matchLoaderToggle();
		intakeGroup.spin(forward);
		driveWithPID(std::sqrt(2275.0) - IMU_distance);
		updateOdometry();
		turnWithPID(-20);
		driveWithPID(5);
		intakeGroup.spin(forward);
		matchLoaderToggle();
		driveWithPID(5);
		intakeGroup.spin(forward);
		wait(3000, msec);
		matchLoaderToggle();
		driveWithPID(8);
		turnWithPID(45);
		blockerToggle();
		driveWithPID(14);
		intakeGroup.spin(forward);
		wait(5000, msec);
	}

	void skillsAuton(void)
	{
		// Generated VEX C++ code for skillsAuton()
		// Copy and paste this into your skillsAuton() function
		// Starting position: (28.00", 85.00") @ 0°
		// IMU offset from center: 7.00" (forward)
		// Calibration offsets: X=4.00", Y=0.50"
		calibrateIMU();
		resetEncoders();
		odometry.reset();
		drivePID.reset();
		turnPID.reset();

		driveWithPID(43);

		turnWithPID(90.0);
		driveWithPID(9.8 + IMU_distance);
		updateOdometry();
		// Waypoint 3: (25.62", 116.14") - Distance: 12.48" (IMU: 49.91")
		driveWithPID(-1 * (12.48 + IMU_distance));
		updateOdometry();
		// Waypoint 4: (29.46", 129.82") - Distance: 14.21" (IMU: 30.05")
		driveWithPID(-1 * (14.21 + IMU_distance));
		updateOdometry();
		// Waypoint 5: (101.7", 130.01") - Distance: 72.24" (IMU: 87.77")
		driveWithPID(-1 * (72.24 + IMU_distance));
		updateOdometry();
		// Waypoint 6: (101.46", 117.53") - Distance: 12.48" (IMU: 31.59")
		driveWithPID(-1 * (12.48 + IMU_distance));
		updateOdometry();
		// Waypoint 7: (104.82", 117.53") - Distance: 3.36" (IMU: 22.44")
		turnWithPID(180.0);
		driveWithPID(3.36 + IMU_distance);
		updateOdometry();
		// Waypoint 8: (98.58", 117.68") - Distance: 6.24" (IMU: 43.66")
		driveWithPID(-1 * (6.24 + IMU_distance));
		updateOdometry();
		Controller1.rumble(".−");
	}

	// --------------------
	// PRE-AUTON
	// --------------------
	void calibrateIMU(void)
	{
		Brain.Screen.clearScreen();
		Brain.Screen.setCursor(1, 1);
		Brain.Screen.print("Calibrating IMU...");
		inertialSensor.calibrate();
		while (inertialSensor.isCalibrating())
		{
			wait(5, msec);
		}
		Brain.Screen.clearLine(1);
		Brain.Screen.setCursor(1, 1);
		Brain.Screen.print("IMU Ready");
		Controller1.Screen.print("IMU Ready");
		wait(20, msec);
	}

	void pre_auton(void)
	{
		// Force default to LEFT auton at startup
		matchAutonSide = true;
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

	void autonomous(void)
	{
		calibrateIMU();
		if (skillsMode)
			skillsAuton();
		else if (matchAutonSide)
			matchAutonRightAveryVersion();
		else
			matchAutonLeft();
	}

	void trackHandPush(void)
	{
		double distanceTraveled;
		bool tracking;

		Brain.Screen.clearScreen();
		Brain.Screen.setCursor(1, 1);
		Brain.Screen.print("Hand Push Mode");
		Brain.Screen.setCursor(2, 1);
		Brain.Screen.print("Tracking...");
		Brain.Screen.setCursor(3, 1);
		Brain.Screen.print("Press Y for result");

		// Reset tracking
		resetEncoders();
		odometry.reset();
		tracking = true;

		// Track until Y button is pressed
		while (tracking)
		{
			// Update odometry
			updateOdometry();

			// Check for result signal
			if (Controller1.ButtonY.pressing())
			{
				tracking = false;
				wait(200, msec);
			}

			wait(10, msec);
		}

		// Show final result
		distanceTraveled = std::sqrt(odometry.x * odometry.x + odometry.y
				* odometry.y);
		Controller1.Screen.clearScreen();
		Controller1.Screen.setCursor(1, 1);
		Controller1.Screen.print("Distance: %.2f in", distanceTraveled);
		Controller1.Screen.setCursor(2, 1);
		Controller1.Screen.print("X: %.2f Y: %.2f", odometry.x, odometry.y);
		Controller1.Screen.setCursor(3, 1);
		Controller1.Screen.print("Heading: %.1f", odometry.heading);
	}

	void usercontrol(void)
	{
		double leftOutput;
		double rightOutput;
		int lastTelemetryMs;
		bool slowTurn;
		int currentMs;

		// IMU already calibrated in pre_auton, no need to recalibrate
		odometry.reset();
		leftOutput = 0.0;
		rightOutput = 0.0;
		lastTelemetryMs = Brain.timer(msec);
		while (1)
		{
			// Toggle between skills and match mode
			if (Controller1.ButtonLeft.pressing())
			{
				skillsMode = !skillsMode;
				Brain.Screen.clearScreen();
				Brain.Screen.setCursor(1, 1);
				Brain.Screen.print("Mode: %s", skillsMode ? "SKILLS" : "MATCH");
				wait(300, msec);
			}
			// Hand push tracking mode (hold L1)
			if (Controller1.ButtonL1.pressing())
			{
				trackHandPush();
				Brain.Screen.clearScreen();
			}
			// Get driver control outputs (ButtonY for slow turn mode)
			slowTurn = Controller1.ButtonY.pressing();
			driverControl.calculate(Controller1, leftOutput, rightOutput,
				slowTurn);
			leftSide.setVelocity(std::abs(leftOutput), percent);
			rightSide.setVelocity(std::abs(rightOutput), percent);
			if (std::abs(leftOutput) < 0.5 && std::abs(rightOutput) < 0.5)
			{
				leftSide.stop(coast);
				rightSide.stop(coast);
			}
			else
			{
				leftSide.spin(leftOutput >= 0.0 ? forward : reverse);
				rightSide.spin(rightOutput >= 0.0 ? forward : reverse);
			}
			// Optional telemetry display every 100ms
			currentMs = Brain.timer(msec);
			if (currentMs - lastTelemetryMs > 100)
			{
				Brain.Screen.setCursor(3, 1);
				Brain.Screen.print("L:%.1f R:%.1f   ", leftOutput, rightOutput);
				lastTelemetryMs = currentMs;
			}
			wait(20, msec);
		}
	}

	int main(void)
	{
		pre_auton();
		Competition.autonomous(autonomous);
		Competition.drivercontrol(usercontrol);
		while (true)
		{
			wait(100, msec);
		}
	}