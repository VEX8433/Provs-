#include "main.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "SubSystems/Intake.hpp"
#include "SubSystems/AutonSelector.hpp"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include <cstddef>

#include "pros/misc.h"
#include "pros/adi.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

// TOF Distance Sensors

// Distance sensor offsets from robot center (mm)
// Positive values = sensor is that many mm away from center in its direction
constexpr float OFFSET_FRONT = 0.0f;  // Front sensor offset (mm from center)
constexpr float OFFSET_LEFT = 0.0f;   // Left sensor offset (mm from center)
constexpr float OFFSET_BACK = 0.0f;   // Back sensor offset (mm from center)

pros::adi::DigitalOut intakeRaise('B');
pros::adi::DigitalOut doinker('A');
pros::adi::DigitalOut tongue('F');
pros::adi::DigitalOut hood('G');
pros::adi::DigitalOut midMech('C');

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-6, -8, 9}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({17, 19, -16}, pros::MotorGearset::blue); // right motors use 200 RPM cartridges

pros::Motor intakeTop(-20, pros::v5::MotorGears::blue);
pros::Motor intakeBottom(10, pros::v5::MotorGears::blue);

pros::Imu inertial(7);
pros::Distance distance(14);
pros::Distance distance2(15);
pros::Rotation tracking(11);

Intake intake(intakeTop, intakeBottom, hood, midMech, intakeRaise, distance, distance2);

lemlib::Drivetrain drivetrain(
	&left_motors, // left motor group
	&right_motors, // right motor group
	10.5, // 10.5 inch track width
	lemlib::Omniwheel::NEW_4, // using new 4" omnis
	300, // drivetrain rpm is 300
	1 // horizontal drift is 0 (for now)
);

lemlib::TrackingWheel vertical_tracking_wheel(&tracking, lemlib::Omniwheel::NEW_2, -0.75);

lemlib::OdomSensors sensors(
	&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
	nullptr, 
	nullptr, 
	nullptr,
	&inertial // inertial sensor
);



// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
											0, // integral gain (kI)
											24, // derivative gain (kD)
											3, // anti windup
											1, // small error range, in inches
											100, // small error range timeout, in milliseconds
											2, // large error range, in inches
											500, // large error range timeout, in milliseconds
											10 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
											0, // integral gain (kI)
											25, // derivative gain (kD)
											3, // anti windup
											1, // small error range, in degrees
											100, // small error range timeout, in milliseconds
											3, // large error range, in degrees
											500, // large error range timeout, in milliseconds
											0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
					lateral_controller, // lateral PID settings
					angular_controller, // angular PID settings
					sensors // odometry sensors
);

/**
 * @brief Reset robot position by moving to a target distance from wall
 * Slows down as it approaches target for accuracy.
 * 
 * @param targetDist Target distance in mm (what the sensor should read)
 * @param useFront true = use front sensor, false = use back sensor
 * @param speed Max motor velocity for adjustment (default 50)
 */



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    
    // Initialize autonomous route selector UI
    AutonSelector::init();
	chassis.calibrate();
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
* This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    // Autonomous selector is already displayed from initialize()
    // Selection remains visible until autonomous or opcontrol starts
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

bool intakeAuton = false;
bool scoreTop = false;
bool scoreMid = false;
bool scoreBot = false;
bool primeMid = false;
bool intakebottom = false;

void changeIntakeState(bool intakeBottom, bool intake, bool scoretop, bool scoremid, bool scorebot, bool primemid){
	intakeAuton = intake;
	scoreTop = scoretop;
	scoreMid = scoremid;
	scoreBot = scorebot;
	primeMid = primemid;
	intakebottom = intakeBottom;
}

void rightside7(){
	// set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
	doinker.set_value(true);

	changeIntakeState(false, true, false, false, false, false);

	chassis.moveToPose(13, 32, 24, 2000, {.maxSpeed = 60});
	pros::delay(1000);
	tongue.set_value(true);
	chassis.turnToHeading(130, 600, {}, false);
	chassis.moveToPoint(37, 6, 1500);
	chassis.turnToHeading(180,  1000);
	chassis.moveToPose(37 ,-20, 180, 1600, {.maxSpeed = 40});
	tongue.set_value(true);
	chassis.moveToPose(37, 30, 180,  1000, {.forwards = false, .minSpeed = 90}, false);
	changeIntakeState(false, false, true, false, false, false);
	pros::delay(1800);
	tongue.set_value(false);
	chassis.moveToPoint(37, 17, 1000, {.minSpeed = 60}, false);
	chassis.turnToHeading(130, 1000);
	chassis.moveToPoint(27, 20, 1000, {.forwards = false});
	chassis.turnToHeading(180, 1000, {}, false);
	doinker.set_value(false);
	chassis.moveToPoint(27, 40, 3000, {.forwards=false,.maxSpeed = 60}, false);
}

void leftside(){
	chassis.setPose(0, 0, 0);
	doinker.set_value(true);

	changeIntakeState(false, true, false, false, false, false);
	chassis.moveToPose( -13, 32, -21, 2000, {.minSpeed = 50});
	pros::delay(1000);
	tongue.set_value(true);
	chassis.moveToPose( -13, 32, -21, 500, {.minSpeed = 50}, false);
	pros::delay(200);
	chassis.turnToHeading(-125, 1000, {}, false); // fix
	changeIntakeState(false, false, false, false, false, true);
	chassis.moveToPose(10, 44, -131, 1000,{.forwards=false, .minSpeed = 60}, false);
	changeIntakeState(false, false, false, true, false, false);
	pros::delay(500);
	changeIntakeState(false, true, false, false, false, false);
	chassis.moveToPoint(-38, 8, 2000, {.maxSpeed = 90});
	pros::delay(200);
	chassis.turnToHeading(180, 1000);
	tongue.set_value(true);
	chassis.moveToPoint(-38, -20, 1500, {.maxSpeed = 40});
	chassis.moveToPoint(-38, 30, 1200, {.forwards=false,.maxSpeed = 80}, false);
	changeIntakeState(false, false, true, false, false, false);
	pros::delay(2000);
	tongue.set_value(false);
	chassis.moveToPoint(-38, 17, 1000, {.minSpeed = 60}, false);
	chassis.turnToHeading(110, 1000);
	chassis.moveToPoint(-46, 19, 1000, {.forwards = false});
	chassis.turnToHeading(180, 1000, {}, false);
	doinker.set_value(false);
	chassis.moveToPoint(-46, 38, 3000, {.forwards=false,.minSpeed = 90}, false);
	// chassis.turnToHeading(150, 3000, {.minSpeed = 50});
}

void rightside4plus3(){
	chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
	doinker.set_value(true);

	changeIntakeState(true, false, false, false, false, false);

	chassis.moveToPose(10, 27, 24, 2000, {.maxSpeed = 60});
	pros::delay(1000);
	tongue.set_value(true);
	chassis.turnToHeading(-45, 1000, {}, false);
	tongue.set_value(false);
	chassis.moveToPoint(-10, 43, 1500, {}, false);
	changeIntakeState(false, false, false, false, true, false);
	pros::delay(700);
	changeIntakeState(false, true, false, false, false, false);

	chassis.moveToPoint(35, 5, 2000, {.forwards = false});

	chassis.turnToHeading(180,  1000);
	chassis.moveToPose(35 ,-20, 180, 1600, {.maxSpeed = 40});
	tongue.set_value(true);
	chassis.moveToPose(35, 30, 180,  1500, {.forwards = false, .minSpeed = 60}, false);
	changeIntakeState(false, false, true, false, false, false);
	pros::delay(1800);
	tongue.set_value(false);
	chassis.moveToPoint(37, 17, 1000, {.minSpeed = 60}, false);
	chassis.turnToHeading(130, 1000);
	chassis.moveToPoint(27, 20, 1000, {.forwards = false});
	chassis.turnToHeading(180, 1000, {}, false);
	doinker.set_value(false);
	chassis.moveToPoint(27, 40, 3000, {.forwards=false,.maxSpeed = 60}, false);
}

void skills(){}
void soloAWP(){}

bool auton = true;
void intakeAutonController(){
	while(auton){
		if(primeMid){
			intakeRaise.set_value(false);
			midMech.set_value(true);
			intakeTop.move_velocity(0);
			intakeBottom.move_velocity(600);
		}
		else if(intakebottom){
			hood.set_value(false);
			intakeRaise.set_value(false);
			intakeTop.move_velocity(0);
			intakeBottom.move_velocity(600);
		}
		else if(scoreBot){
			intakeRaise.set_value(true);
			intakeTop.move_velocity(-600);
			intakeBottom.move_velocity(-600);
		}
		else if(intakeAuton){
			intakeRaise.set_value(false);
			intakeBottom.move_velocity(600);
			hood.set_value(false);
			midMech.set_value(false);

			if(distance2.get_distance() <= 30 && distance.get_distance() <= 30){
				pros::delay(100);
				intakeTop.move_velocity(0);
			}
			else{
				intakeTop.move_velocity(600);
			}
		}
		else if(scoreTop){
			intakeRaise.set_value(false);
			hood.set_value(true);
			midMech.set_value(false);
			intakeTop.move_velocity(600);
			intakeBottom.move_velocity(600);
		}
		else if(scoreMid){
			intakeRaise.set_value(false);
			intakeTop.move_velocity(600);
			intakeBottom.move_velocity(600);
			hood.set_value(false);
			midMech.set_value(true);
		}
		else{
			intakeRaise.set_value(false);
			intakeTop.move_velocity(0);
			intakeBottom.move_velocity(0);
		}
	}
}

void autonomous(){
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	auton = true;
	pros::Task intakeAutonControl(intakeAutonController);
	// rightside7();
	// rightside4plus3();
	// leftside();
	switch (AutonSelector::getSelectedRoute()) {
		case AutonRoute::SKILLS:
			skills();
			break;
		case AutonRoute::RIGHT_SIDE:
			rightside7();
			break;
		case AutonRoute::LEFT_SIDE:
			leftside();
			break;
		case AutonRoute::SOLO_AWP:
			soloAWP();
			break;
		case AutonRoute::DO_NOTHING:
			// Do nothing - robot stays still
			break;
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void driveTelop(int leftY, int rightX){// 127, -127
    // https://www.desmos.com/calculator/lve1dfzzku
    double t = 5;
    float forward = 4.7244 * leftY;
    // float forward = (exp(-(t/10))+exp((abs(leftY)-127)/10)*(1- exp(-(t/10))))*leftY * 4.7244;
    // float turn = 3.5 * rightX;//4.7244 * rightX
    float turn = (exp(-(t/10))+exp((abs(rightX)-127)/10)*(1- exp(-(t/10))))*rightX * 4;

    left_motors.move_velocity(forward + turn);
    right_motors.move_velocity(forward - turn);
}

void opcontrol() {
	// Clean up selector UI when entering driver control
	AutonSelector::destroy();


	auton = false;
	
	bool doinkerToggle = false;
	bool tongueToggle = false;
	bool backdoinkerToggle = false;
	bool doubleParkToggle = false;

	
	// pros::Task thing(intake.telOP(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1), master.get_digital(pros::E_CONTROLLER_DIGITAL_L1), master.get_digital(pros::E_CONTROLLER_DIGITAL_X), master.get_digital(pros::E_CONTROLLER_DIGITAL_R2), master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN), master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)));
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

	while(true){
		//Function in chassis class to move according to joystick inputs
		driveTelop(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 
		master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

		intake.telOP(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1), master.get_digital(pros::E_CONTROLLER_DIGITAL_L1), 
		master.get_digital(pros::E_CONTROLLER_DIGITAL_X), master.get_digital(pros::E_CONTROLLER_DIGITAL_B), master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            tongueToggle = !tongueToggle;
            tongue.set_value(tongueToggle);
        }
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            doinkerToggle = !doinkerToggle;
            doinker.set_value(doinkerToggle);
        }
		pros::delay(20);
	}
}
