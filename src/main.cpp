#include "Chassis.h"
#include "DriverControl.h"
#include "vex.h"
#include <cmath>
#include <vector>

using namespace	vex;

competition		Competition;

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

digital_out		MatchLoader(Brain.ThreeWirePort.C);
digital_out		DeScorePiston(Brain.ThreeWirePort.B);
digital_out		middleScore(Brain.ThreeWirePort.A);

vex::inertial inertialSensor = vex::inertial(PORT21);
Chassis			chassis(leftSide, rightSide, inertialSensor);

// Set this variable to choose which auton to run:
// 0 = leftAuton, 1 = rightAuton, 2 = skillsAuton, 3 = skillsAuton2
int				selectedAuton = 2;

// Forward declarations for functions defined later
void			calibrateIMU(void);
bool			matchAutonRight = false;

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

bool			matchLoaderExtended = false;
bool			deScorePistonExtended = false;
bool			middleScoreExtended = false;
int				intakeOutputVelocity = 50;

void	matchLoaderToggle(void)
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

// drive with optional inline PID tuning
void	drive(double dist, int velocity = 100, int timeout = 5000,
		double kP = NAN, double kI = NAN, double kD = NAN)
{
	chassis.drive(dist, velocity, timeout, kP, kI, kD);
}
// turn with optional inline PID tuning
void	turn(double heading, int timeout = 3000, double kP = NAN,
		double kI = NAN, double kD = NAN)
{
	chassis.turn(heading, timeout, kP, kI, kD);
}

void	resetEncoders(void)
{
	chassis.left.resetPosition();
	chassis.right.resetPosition();
	intakeGroup.resetPosition();
	chassis.odom.reset();
}

struct			SplinePoint
{
	double		x;
	double		y;
	double heading; // degrees
};

// Generate cubic spline points between start and end (optionally with headings)
std::vector<SplinePoint> generateSpline(double startX, double startY,
	double startHeading, double endX, double endY, double endHeading,
	int numPoints = 50)
{
	double	startTheta;
	double	endTheta;
	double	dx;
	double	dy;
	double	dist;
	double	startDx;
	double	startDy;
	double	endDx;
	double	endDy;
	double	t;
	double	h00;
	double	h10;
	double	h01;
	double	h11;
	double	x;
	double	y;
	double	heading;

	std::vector<SplinePoint> points;
	// Convert headings to radians
	startTheta = startHeading * M_PI / 180.0;
	endTheta = endHeading * M_PI / 180.0;
	// Cubic Hermite spline coefficients
	dx = endX - startX;
	dy = endY - startY;
	dist = std::sqrt(dx * dx + dy * dy);
	startDx = std::cos(startTheta) * dist;
	startDy = std::sin(startTheta) * dist;
	endDx = std::cos(endTheta) * dist;
	endDy = std::sin(endTheta) * dist;
	for (int i = 0; i <= numPoints; ++i)
	{
		t = (double)i / numPoints;
		// Hermite basis functions
		h00 = 2 * t * t * t - 3 * t * t + 1;
		h10 = t * t * t - 2 * t * t + t;
		h01 = -2 * t * t * t + 3 * t * t;
		h11 = t * t * t - t * t;
		x = h00 * startX + h10 * startDx + h01 * endX + h11 * endDx;
		y = h00 * startY + h10 * startDy + h01 * endY + h11 * endDy;
		heading = startHeading + t * (endHeading - startHeading);
		points.push_back({x, y, heading});
	}
	return (points);
}

struct			Point
{
	double		x;
	double		y;
};

// Hermite spline point generator
Point	getHermitePoint(double t, Point pStart, double angleStart, Point pEnd,
		double angleEnd, double magnitude)
{
	double	vStart_x;
	double	vStart_y;
	double	vEnd_x;
	double	vEnd_y;
	double	t2;
	double	t3;
	double	h00;
	double	h10;
	double	h01;
	double	h11;
	double	x;
	double	y;

	vStart_x = sin(angleStart * M_PI / 180.0) * magnitude;
	vStart_y = cos(angleStart * M_PI / 180.0) * magnitude;
	vEnd_x = sin(angleEnd * M_PI / 180.0) * magnitude;
	vEnd_y = cos(angleEnd * M_PI / 180.0) * magnitude;
	t2 = t * t;
	t3 = t2 * t;
	h00 = 2 * t3 - 3 * t2 + 1;
	h10 = t3 - 2 * t2 + t;
	h01 = -2 * t3 + 3 * t2;
	h11 = t3 - t2;
	x = h00 * pStart.x + h10 * vStart_x + h01 * pEnd.x + h11 * vEnd_x;
	y = h00 * pStart.y + h10 * vStart_y + h01 * pEnd.y + h11 * vEnd_y;
 return Point{x, y};
}

// Calculates a point on a quadratic spline based on progress 't' (0 to 1)
Point	getSplinePoint(double t, Point p0, Point p1, Point p2)
{
	double	x;
	double	y;

	x = (1 - t) * (1 - t) * p0.x + 2 * (1 - t) * t * p1.x + t * t * p2.x;
	y = (1 - t) * (1 - t) * p0.y + 2 * (1 - t) * t * p1.y + t * t * p2.y;
 return Point{x, y};
}

// Example: drive along a spline path (prints target points)
void	driveSplinePath(void)
{
	Point	start;
	Point	end;
	Point	target;

	start = {0, 0};
	Point control = {2, 2}; // Curvature point
	end = {5, 5};
	for (double t = 0; t <= 1.0; t += 0.05)
	{
		target = getSplinePoint(t, start, control, end);
 		// Add your movement logic here (e.g., Pure Pursuit or Stanley Controller)
		printf("Target: X: %.2f, Y: %.2f\n", target.x, target.y);
	}
}

// Simple drive-to-point function using odometry and proportional control
void	driveToPoint(double targetX, double targetY, double maxSpeed = 50,
		double tolerance = 0.5, int timeout = 2000)
{
	const double	dt = 0.01;
	int				elapsed;
	double			dx;
	double			dy;
	double			distance;
	double			targetAngle;
	double			headingError;
	double			driveOut;
	double			turnOut;
	double			leftOut;
	double			rightOut;

	elapsed = 0;
	while (elapsed < timeout)
	{
		chassis.updateOdometry();
		dx = targetX - chassis.odom.x;
		dy = targetY - chassis.odom.y;
		distance = std::sqrt(dx * dx + dy * dy);
		targetAngle = std::atan2(dy, dx) * 180.0 / M_PI;
		headingError = Chassis::normalizeAngle(targetAngle
				- chassis.odom.heading);
		// Simple proportional control
		driveOut = std::fmax(std::fmin(distance * 10, maxSpeed), -maxSpeed);
		turnOut = std::fmax(std::fmin(headingError * 2, 40.0), -40.0);
		leftOut = driveOut + turnOut;
		rightOut = driveOut - turnOut;
		chassis.left.setVelocity(std::abs(leftOut), vex::percent);
		chassis.right.setVelocity(std::abs(rightOut), vex::percent);
		if (leftOut >= 0)
			chassis.left.spin(vex::forward);
		else
			chassis.left.spin(vex::reverse);
		if (rightOut >= 0)
			chassis.right.spin(vex::forward);
		else
			chassis.right.spin(vex::reverse);
		if (distance < tolerance)
			break ;
		vex::wait(dt * 1000, vex::msec);
		elapsed += int(dt * 1000);
	}
	chassis.left.stop();
	chassis.right.stop();
}

// New function: driveHermiteSplinePath
void	driveHermiteSplinePath(Point startPos, double startHeading,
		Point endPos, double endHeading, double tightness, int numPoints = 20)
{
	Point	target;
		const double dt = 0.01;
	int		elapsed;
	double	tolerance;
	double	maxSpeed;
	double	dx;
	double	dy;
	double	distance;
	double	targetAngle;
	double	headingError;
	double	driveOut;
	double	turnOut;
	double	leftOut;
	double	rightOut;

	for (double t = 0; t <= 1.0; t += 1.0 / numPoints)
	{
		target = getHermitePoint(t, startPos, startHeading, endPos, endHeading,
				tightness);
		// Simple proportional control to drive to each point
		elapsed = 0;
		tolerance = 0.5;
		maxSpeed = 50;
		while (elapsed < 1000)
		{ // 1s timeout per point
			chassis.updateOdometry();
			dx = target.x - chassis.odom.x;
			dy = target.y - chassis.odom.y;
			distance = std::sqrt(dx * dx + dy * dy);
			targetAngle = std::atan2(dy, dx) * 180.0 / M_PI;
			headingError = Chassis::normalizeAngle(targetAngle
					- chassis.odom.heading);
			driveOut = std::fmax(std::fmin(distance * 10, maxSpeed), -maxSpeed);
			turnOut = std::fmax(std::fmin(headingError * 2, 40.0), -40.0);
			leftOut = driveOut + turnOut;
			rightOut = driveOut - turnOut;
			chassis.left.setVelocity(std::abs(leftOut), vex::percent);
			chassis.right.setVelocity(std::abs(rightOut), vex::percent);
			if (leftOut >= 0)
				chassis.left.spin(vex::forward);
			else
				chassis.left.spin(vex::reverse);
			if (rightOut >= 0)
				chassis.right.spin(vex::forward);
			else
				chassis.right.spin(vex::reverse);
			if (distance < tolerance)
				break ;
			vex::wait(dt * 1000, vex::msec);
			elapsed += int(dt * 1000);
		}
		chassis.left.stop();
		chassis.right.stop();
	}
}

// Simple API: curveDrive(startX, startY, startHeading, endX, endY, endHeading, tightness, numPoints)
void curveDrive(double startX, double startY, double startHeading,
	double endX, double endY, double endHeading, bool reverse = false,
	double tightness = 20, int numPoints = 20)
{
	Point target;
	double dx;
	double dy;
	double distance;
	double targetAngle;
	double headingError;
	double driveOut;
	double turnOut;
	double leftOut;
	double rightOut;

	double direction = reverse ? -1.0 : 1.0;
	for (double t = 0; t <= 1.0; t += 1.0 / numPoints)
	{
		double param = reverse ? (1.0 - t) : t;
		target = getHermitePoint(param, {startX, startY}, startHeading, {endX,
			endY}, endHeading, tightness);
		chassis.updateOdometry();
		dx = target.x - chassis.odom.x;
		dy = target.y - chassis.odom.y;
		distance = std::sqrt(dx * dx + dy * dy);
		targetAngle = std::atan2(dy, dx) * 180.0 / M_PI;
		headingError = Chassis::normalizeAngle(targetAngle
			- chassis.odom.heading);
		driveOut = direction * std::fmin(distance * 10, 50);
		turnOut = direction * std::fmax(std::fmin(headingError * 2, 40.0), -40.0);
		leftOut = driveOut + turnOut;
		rightOut = driveOut - turnOut;
		chassis.left.setVelocity(std::abs(leftOut), vex::percent);
		chassis.right.setVelocity(std::abs(rightOut), vex::percent);
		chassis.left.spin(leftOut >= 0 ? vex::forward : vex::reverse);
		chassis.right.spin(rightOut >= 0 ? vex::forward : vex::reverse);
		vex::wait(20, msec);
	}
	chassis.left.stop();
	chassis.right.stop();
}


// --------------------
// AUTON FUNCTIONS
// --------------------
// Forward declaration for skills autonomous
void			skillsAuton(void);
void			backupSkills(void);

void	matchAutonRightRoutine(void)
{
	resetEncoders();
	chassis.reset();
	matchLoaderToggle();
	drive(45, 75, 5000);
	intakeMotor.spin(reverse);
	turn(90.0);
	drive(60, 75, 75, 4000);
		// Drives 60 units while rotating 75 degrees at 75% speed, 4s timeout
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

void	matchAutonLeftRoutine(void)
{
	resetEncoders();
	chassis.reset();
	drive(32);
	wait(500, msec);
	turn(105);
	matchLoaderToggle();
	drive(45);
	turn(150);
	drive(4);
	wait(1500, msec);
	drive(-40);
	drive(10);
	turn(240);
	drive(-15);
	turn(150);
	drive(-10);
	Controller1.rumble(".-");
}

void	skillsAuton(void)
{
	calibrateIMU();
	resetEncoders();
	chassis.reset();
	// intakeMotor.spin(reverse);
	// matchLoaderToggle();
	// drive(60, 30, 4000, 0.75, 0.01, 0.5);
	// matchLoaderToggle();
	// drive(55.0, 100, 2000, 3.5, 0.5, 0.01);
	// turn(-50.0);
	// drive(20.0);
	// wait(2000, msec);
	drive(20.0);
	turn(-90);

	Controller1.rumble(".-");
}

void	skillsAuton2(void)
{
	calibrateIMU();
	resetEncoders();
	chassis.reset();
	curveDrive(0, 0, 0, 30, 30, 90, false, 50, 50);
	turn(90.0);
	Controller1.rumble(".-");
}

// --------------------
// PRE-AUTON
// --------------------
void	calibrateIMU(void)
{
	Brain.Screen.clearScreen();
	Brain.Screen.setCursor(1, 1);
	Brain.Screen.print("Calibrating IMU...");
	inertialSensor.calibrate();
	while (inertialSensor.isCalibrating())
	{
		vex::wait(1, msec);
	}
	Brain.Screen.clearLine(1);
	Brain.Screen.setCursor(1, 1);
	Brain.Screen.print("IMU Ready");
	Controller1.Screen.print("IMU Ready");
}

void	pre_auton(void)
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

void	autonomous(void)
{
	switch (selectedAuton)
	{
	case 0:
		matchAutonLeftRoutine();
		break ;
	case 1:
		matchAutonRightRoutine();
		break ;
	case 2:
		skillsAuton();
		break ;
	case 3:
		skillsAuton2();
		break ;
	default:
		matchAutonLeftRoutine();
		break ;
	}
}

void	usercontrol(void)
{
	double	forward;
	double	turn;
	double	leftPower;
	double	rightPower;

	while (true)
	{
		// Split arcade: left stick Y for forward/back
		forward = Controller1.Axis3.position();
		turn = Controller1.Axis1.position();
		leftPower = forward + turn;
		rightPower = forward - turn;
		chassis.left.setVelocity(leftPower, percent);
		chassis.right.setVelocity(rightPower, percent);
		// Stop motors if no input
		if (std::abs(leftPower) < 0.5 && std::abs(rightPower) < 0.5)
		{
			chassis.left.stop(coast);
			chassis.right.stop(coast);
		}
		else
		{
			chassis.left.spin((leftPower >= 0.0) ? vex::forward : vex::reverse);
			chassis.right.spin((rightPower >= 0.0) ? vex::forward : vex::reverse);
		}
	}
}

int	main(void)
{
	pre_auton();
	Competition.autonomous(autonomous);
	Competition.drivercontrol(usercontrol);
}