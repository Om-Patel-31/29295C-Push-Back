#include "DriverControl.h"
#include "Chassis.h"
#include "vex.h"
#include <cmath>


using namespace	vex;

competition Competition;

// DRIVER CONTROL CONFIGURATION - Change this to switch control schemes
DriverControl	driverControl(ControlType::ARCADE, 10, 2.0, 50, 40);

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

digital_out		MatchLoader(Brain.ThreeWirePort.B);
digital_out		DeScorePiston(Brain.ThreeWirePort.A);
digital_out		middleScore(Brain.ThreeWirePort.C);

vex::inertial inertialSensor = vex::inertial(PORT21);
Chassis chassis(leftSide, rightSide, inertialSensor);


// Set this variable to choose which auton to run:
// 0 = leftAuton, 1 = rightAuton, 2 = skillsAuton, 3 = skillsAuton2
int selectedAuton = 3;


// Forward declarations for functions defined later
void calibrateIMU(void);
bool matchAutonRight = false;

void intakeForwardPressed(void)
{
	intakeMotor.spin(forward);
}
void intakeForwardReleased(void)
{
	if (!Controller1.ButtonR2.pressing())
	{
		intakeMotor.stop();
	}
}
void intakeReversePressed(void)
{
	intakeMotor.spin(reverse);
}
void intakeReverseReleased(void)
{
	if (!Controller1.ButtonR1.pressing())
		intakeMotor.stop();
}

void outputForwardPressed(void)
{
	outputMotor.spin(forward);
}
void outputForwardReleased(void)
{
	if (!Controller1.ButtonL2.pressing())
	{
		outputMotor.stop();
	}
}
void outputReversePressed(void)
{
	outputMotor.spin(reverse);
}
void outputReverseReleased(void)
{
	if (!Controller1.ButtonL1.pressing())
	{
		outputMotor.stop();
	}
}

void SpinBothForwardPressed(void)
{
	intakeForwardPressed();
	outputForwardPressed();
}

void SpinBothForwardReleased(void)
{
	intakeForwardReleased();
	outputForwardReleased();
}

void SpinBothReversePressed(void)
{
	intakeReversePressed();
	outputReversePressed();
}

void SpinBothReverseReleased(void)
{
	intakeReverseReleased();
	outputReverseReleased();
}

bool			matchLoaderExtended = false;
bool			deScorePistonExtended = false;
bool			middleScoreExtended = false;
int			intakeOutputVelocity = 50;

void matchLoaderToggle(void)
{
	matchLoaderExtended = !matchLoaderExtended;
	MatchLoader.set(matchLoaderExtended);
	Brain.Screen.clearLine(4);
	Brain.Screen.setCursor(4, 1);
	Brain.Screen.print("MatchLD: %s", matchLoaderExtended ? "EXT" : "CL");
}

void	deScorePistonToggle(void)
{
	deScorePistonExtended = !deScorePistonExtended;
	DeScorePiston.set(deScorePistonExtended);
	Brain.Screen.clearLine(5);
	Brain.Screen.setCursor(5, 1);
	Brain.Screen.print("DeSc: %s", deScorePistonExtended ? "EXT" : "CL");
}

void	middleScoreToggle(void)
{
	middleScoreExtended = !middleScoreExtended;
	middleScore.set(middleScoreExtended);
	Brain.Screen.clearLine(6);
	Brain.Screen.setCursor(6, 1);
	Brain.Screen.print("MidSc: %s", middleScoreExtended ? "EXT" : "CL");
}

void	toggleIntakeOutputVelocity(void)
{
	intakeOutputVelocity = (intakeOutputVelocity == 50) ? 100 : 50;
	intakeMotor.setVelocity(intakeOutputVelocity, percent);
	outputMotor.setVelocity(intakeOutputVelocity, percent);
	Brain.Screen.clearLine(7);
	Brain.Screen.setCursor(7, 1);
	Brain.Screen.print("Velocity: %d%%", intakeOutputVelocity);
}

template <typename T>
static T clamp(T value, T minVal, T maxVal)
{
	if (value > maxVal)
		return (maxVal);
	if (value < minVal)
		return (minVal);
	return (value);
}

static double	normalizeAngleDeg(double angle)
{
	while (angle > 180.0)
		angle -= 360.0;
	while (angle < -180.0)
		angle += 360.0;
	return (angle);
}

void	increaseExpo(void)
{
	double currentExpo;

	currentExpo = driverControl.getExponent();
	if (currentExpo < 5.0)
	{
		driverControl.setExponent(std::round((currentExpo + 0.1) * 100.0) / 100.0);
		Controller1.Screen.clearLine(3);
		Controller1.Screen.setCursor(3, 1);
		Controller1.Screen.print("Expo: %.2f", driverControl.getExponent());
	}
}

void decreaseExpo(void)
{
	double currentExpo;

	currentExpo = driverControl.getExponent();
	if (currentExpo > 1.0)
	{
		driverControl.setExponent(std::round((currentExpo - 0.1) * 100.0) / 100.0);
		Controller1.Screen.clearLine(3);
		Controller1.Screen.setCursor(3, 1);
		Controller1.Screen.print("Expo: %.2f", driverControl.getExponent());
	}
}

void increaseTurnSens(void)
{
	int currentSens;

	currentSens = driverControl.getTurnSensitivity();
	if (currentSens < 100)
	{
		driverControl.setTurnSensitivity(currentSens + 5);
		Brain.Screen.clearLine(3);
		Brain.Screen.setCursor(3, 1);
		Brain.Screen.print("Turn Sens: %d", driverControl.getTurnSensitivity());
	}
}

void decreaseTurnSens(void)
{
	int currentSens;

	currentSens = driverControl.getTurnSensitivity();
	if (currentSens > 10)
	{
		driverControl.setTurnSensitivity(currentSens - 5);
		Brain.Screen.clearLine(3);
		Brain.Screen.setCursor(3, 1);
		Brain.Screen.print("Turn Sens: %d", driverControl.getTurnSensitivity());
	}
}


// New high-level chassis API wrappers for compatibility
// drive with optional inline PID tuning
void drive(double dist, int velocity = 100, int timeout = 5000,
				  double kP = NAN, double kI = NAN, double kD = NAN) {
	chassis.drive(dist, velocity, timeout, kP, kI, kD);
}
// turn with optional inline PID tuning
void turn(double heading, int timeout = 3000,
				 double kP = NAN, double kI = NAN, double kD = NAN) {
	chassis.turn(heading, timeout, kP, kI, kD);
}
// moveToPoint with optional heading and inline PID tuning
void moveToPoint(
	double x, double y, double heading = NAN, int velocity = 100, int timeout = 5000,
	double drive_kP = NAN, double drive_kI = NAN, double drive_kD = NAN,
	double turn_kP = NAN, double turn_kI = NAN, double turn_kD = NAN,
	double curve_kP = NAN, double curve_kI = NAN, double curve_kD = NAN
) {
	chassis.moveToPoint(x, y, heading, velocity, timeout,
		drive_kP, drive_kI, drive_kD,
		turn_kP, turn_kI, turn_kD,
		curve_kP, curve_kI, curve_kD);
}
void resetEncoders(void) {
    chassis.left.resetPosition();
    chassis.right.resetPosition();
    intakeGroup.resetPosition();
    chassis.odom.reset();
}

// --------------------
// AUTON FUNCTIONS
// --------------------
// Forward declaration for skills autonomous
void			skillsAuton(void);
void			backupSkills(void);


void matchAutonRightRoutine(void) {
    calibrateIMU();
    resetEncoders();
    chassis.reset();
    // Example: drive forward, curve to point, turn
    matchLoaderToggle();
    drive(45, 75, 5000);
    intakeMotor.spin(reverse);
    turn(90.0);
    moveToPoint(60, 30, 75, 4000); // Example curve
    wait(2000, msec);
    intakeMotor.stop();
    drive(-30);
    intakeMotor.spin(reverse);
    outputMotor.spin(reverse);
    wait(2000, msec);
    intakeMotor.stop();
    outputMotor.stop();
    matchLoaderToggle();
    Controller1.rumble(".-");
}


void matchAutonLeftRoutine(void) {
    calibrateIMU();
    resetEncoders();
    chassis.reset();
    matchLoaderToggle();
    drive(43.5, 75, 5000);
    turn(-90.0);
    intakeMotor.setVelocity(100, percent);
    intakeMotor.spin(reverse);
    moveToPoint(10, 40, 75, 3000); // Example curve
    wait(1500, msec);
    intakeMotor.stop();
    drive(-35);
    intakeMotor.spin(reverse);
    outputMotor.spin(reverse);
    wait(3000, msec);
    outputMotor.stop();
    intakeMotor.stop();
    Controller1.rumble(".-");
}


void skillsAuton(void) {
    calibrateIMU();
    resetEncoders();
    chassis.reset();
    intakeMotor.setVelocity(100, percent);
    intakeMotor.spin(reverse);
    drive(60, 100, 4000, 2.5, 0.01, 0.5);
	matchLoaderToggle();
    drive(30, 100, 5000, 5, 0.1, 1.0);
	matchLoaderToggle();
    Controller1.rumble(".-");
}

void skillsAuton2(void) {
    calibrateIMU();
    resetEncoders();
    chassis.reset();
	intakeMotor.setVelocity(100, percent);

	matchLoaderToggle();
	drive(44, 75, 5000);
    turn(90.0);
	intakeMotor.spin(reverse);
	drive(12, 60, 5000);
	
	matchLoaderToggle();
	intakeMotor.stop();
	drive(-15.0);
	// turn(-15);
	// drive(10);
    // turn(-90.0);
	// drive(120);
    // turn(-135.0);
	// drive(10);
    // turn(-90.0);
	// drive(-25);
	// intakeMotor.spin(reverse);
	// outputMotor.spin(reverse);
	// wait(4000);
	// intakeMotor.stop();
	// outputMotor.stop();
	// matchLoaderToggle();
	// drive(30);
	// intakeMotor.spin(reverse);


    Controller1.rumble(".-");
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
		vex::wait(5, msec);
	}
	Brain.Screen.clearLine(1);
	Brain.Screen.setCursor(1, 1);
	Brain.Screen.print("IMU Ready");
	Controller1.Screen.print("IMU Ready");
	vex::wait(20, msec);
}

void pre_auton(void)
{
	// Force default to LEFT auton at startup
	matchAutonRight = false;
	matchLoaderExtended = false;
	MatchLoader.set(false);
	leftSide.setStopping(vex::brakeType::coast);
	rightSide.setStopping(vex::brakeType::coast);
	intakeMotor.setStopping(vex::brakeType::brake);
	outputMotor.setStopping(vex::brakeType::brake);
	intakeMotor.setVelocity(50, percent);
	outputMotor.setVelocity(100, percent);
	calibrateIMU();
	Controller1.ButtonR1.pressed(intakeForwardPressed);
	Controller1.ButtonR1.released(intakeForwardReleased);
	Controller1.ButtonR2.pressed(intakeReversePressed);
	Controller1.ButtonR2.released(intakeReverseReleased);
	Controller1.ButtonL1.pressed(outputForwardPressed);
	Controller1.ButtonL1.released(outputForwardReleased);
	Controller1.ButtonL2.pressed(outputReversePressed);
	Controller1.ButtonL2.released(outputReverseReleased);
	Controller1.ButtonX.pressed(matchLoaderToggle);
	Controller1.ButtonA.pressed(deScorePistonToggle);
	Controller1.ButtonB.pressed(middleScoreToggle);
	Controller1.ButtonUp.pressed(toggleIntakeOutputVelocity);
}

void	autonomous(void) {
	switch (selectedAuton) {
		case 0:
			matchAutonLeftRoutine();
			break;
		case 1:
			matchAutonRightRoutine();
			break;
		case 2:
			skillsAuton();
			break;
		case 3:
			skillsAuton2();
			break;
		default:
			matchAutonLeftRoutine();
			break;
	}
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
	chassis.odom.reset();
	tracking = true;

	// Track until Y button is pressed
	while (tracking)
	{
		// Update odometry
		chassis.updateOdometry();
		// Check for result signal
		if (Controller1.ButtonY.pressing())
		{
			tracking = false;
			vex::wait(200, msec);
		}

		vex::wait(10, msec);
	}

	// Show final result
	    distanceTraveled = std::sqrt(chassis.odom.x * chassis.odom.x + chassis.odom.y * chassis.odom.y);
	    Controller1.Screen.clearScreen();
	    Controller1.Screen.setCursor(1, 1);
	    Controller1.Screen.print("Distance: %.2f in", distanceTraveled);
	    Controller1.Screen.setCursor(2, 1);
	    Controller1.Screen.print("X: %.2f Y: %.2f", chassis.odom.x, chassis.odom.y);
	    Controller1.Screen.setCursor(3, 1);
	    Controller1.Screen.print("Heading: %.1f", chassis.odom.heading);
}


void usercontrol(void) {
	while (true) {
		// Split arcade: left stick Y for forward/back, right stick X for turning
		double forward = Controller1.Axis3.position();
		double turn = Controller1.Axis1.position();
		double leftPower = forward + turn;
		double rightPower = forward - turn;
		chassis.left.setVelocity(leftPower, percent);
		chassis.right.setVelocity(rightPower, percent);
		// Stop motors if no input
		if (std::abs(leftPower) < 0.5 && std::abs(rightPower) < 0.5) {
			chassis.left.stop(coast);
			chassis.right.stop(coast);
		} else {
			chassis.left.spin((leftPower >= 0.0) ? vex::forward : vex::reverse);
			chassis.right.spin((rightPower >= 0.0) ? vex::forward : vex::reverse);
		}
		vex::wait(20, vex::msec);
	}
}

// ============================================
// BACKUP: Original skillsMode and skillsAuton
// ============================================

// Backup skillsAuton function
void	backupSkills(void)
{
	calibrateIMU();
	resetEncoders();
	chassis.odom.reset();
	chassis.drivePID.reset();
	chassis.turnPID.reset();

	intakeMotor.spin(fwd);
	drive(70, 50, 5000);
}

int main(void)
{
	pre_auton();
	Competition.autonomous(autonomous);
	Competition.drivercontrol(usercontrol);
	while (true)
	{
		vex::wait(100, msec);
	}
}