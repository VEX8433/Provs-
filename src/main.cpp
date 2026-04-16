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
pros::Rotation vert_tracking(-4);
pros::Rotation hor_tracking(-1);


Intake intake(intakeTop, intakeBottom, hood, midMech, intakeRaise, distance, distance2);

lemlib::Drivetrain drivetrain(
	&left_motors, // left motor group
	&right_motors, // right motor group
	10.5, // 10.5 inch track width
	lemlib::Omniwheel::NEW_4, // using new 4" omnis
	300, // drivetrain rpm is 300
	1 // horizontal drift is 0 (for now)
);

lemlib::TrackingWheel vertical_tracking_wheel(&vert_tracking, lemlib::Omniwheel::NEW_2, -0.75);
lemlib::TrackingWheel horizontal_tracking_wheel(&hor_tracking, lemlib::Omniwheel::NEW_2, -3.25);


lemlib::OdomSensors sensors(
	&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
	nullptr, 
	&horizontal_tracking_wheel, 
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
    
    // Initialize autonomous route selector UI
    //AutonSelector::init();
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
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
bool scoreMidSlow = false;
bool scoreLowSlow = false;

void changeIntakeState(bool intakeBottom, bool intake, bool scoretop, bool scoremid, bool scorebot, bool primemid){
	intakeAuton = intake;
	scoreTop = scoretop;
	scoreMid = scoremid;
	scoreBot = scorebot;
	primeMid = primemid;
	intakebottom = intakeBottom;
}

void scoreSlow(bool mid, bool low){
	scoreMidSlow=mid;
	scoreLowSlow=low;
}

void rightside7(){
	// set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
	doinker.set_value(true);

	changeIntakeState(false, true, false, false, false, false);

	chassis.moveToPose(13, 32, 24, 2000, {.minSpeed = 60});
	pros::delay(500);
	tongue.set_value(true);
	chassis.waitUntilDone();
	pros::delay(200);
	chassis.turnToHeading(130, 600, {}, false);
	
	
	pros::delay(20000000);
	chassis.moveToPoint(37, 6, 1000, {.earlyExitRange = 1});
	chassis.turnToHeading(180,  300);


	chassis.moveToPose(37 ,-20, 180, 1600, {.maxSpeed = 80, .minSpeed = 50});
	tongue.set_value(true);
	chassis.moveToPose(37, 30, 180,  1000, {.forwards = false, .minSpeed = 70}, false);
	changeIntakeState(false, false, true, false, false, false);
	pros::delay(1800);
	tongue.set_value(false);
	
	chassis.moveToPoint(28, 8, 1000);
	chassis.turnToHeading(180, 300, {}, false);
	doinker.set_value(false);
	chassis.moveToPoint(28, 35, 3000, {.forwards=false,.maxSpeed = 60}, false);
}

void elim_left(){
	// chassis.moveToPose(9.5, 43.5) = midgoal point
	chassis.setPose(-10, 40, -135);
	// chassis.moveToPose(-45.5, 32.5, 0, 1500, {.minSpeed = 50, .earlyExitRange = 10}, false);
	// chassis.moveToPoint(-45.5, 36, 1500, {.maxSpeed = 90});
	// doinker.set_value(false);
	// chassis.turnToHeading(145, 1000);
	chassis.moveToPose(-32, 32, -90, 3000, {.minSpeed = 120, .earlyExitRange=3}, false);
	chassis.swingToHeading(0, DriveSide::RIGHT, 750, {.minSpeed = 90});
	// chassis.moveToPoint(-3, 15, 800, {.earlyExitRange = 0}, false);
	// //pros::delay(200);
	// chassis.turnToHeading(0, 1000, {}, false);
	// doinker.set_value(false);
	chassis.moveToPoint(-37, 39, 1000, {.minSpeed = 120});
	chassis.setPose(-37, 39, 0);
	chassis.moveToPoint(-35.5, 57, 350, {.minSpeed = 120});
	chassis.turnToHeading(45, 250);

}

void leftside(){
	chassis.setPose(0, 0, 0);
	doinker.set_value(true);

	changeIntakeState(false, true, false, false, false, false);
	chassis.moveToPose( -10, 26, -21, 1500, {.minSpeed = 60});
	pros::delay(600);
	tongue.set_value(true);
	chassis.waitUntilDone();
	//chassis.moveToPose( -13.5, 31.5, -21, 500, {.minSpeed = 60}, false);
	pros::delay(200);
	chassis.turnToHeading(-125, 250, {}, false); // fix
	//back into midgoal
	chassis.moveToPose(9.5, 43.5, -131, 1000,{.forwards=false, .minSpeed = 60});
	pros::delay(500);
	changeIntakeState(false, false, false, false, false, true);
	chassis.waitUntilDone();

	changeIntakeState(false, false, false, false, false, false);
	scoreSlow(true, false);
	pros::delay(800);
	scoreSlow(false, false);
	changeIntakeState(false, true, false, false, false, false);
	chassis.moveToPoint(-37, 3, 1500, {.maxSpeed = 90});
	pros::delay(200);

	chassis.turnToHeading(180, 250);
	tongue.set_value(true);
	// go into match load
	chassis.moveToPoint(-37, -20, 1500, {.maxSpeed = 50});
	chassis.moveToPoint(-37, 30, 1200, {.forwards=false,.maxSpeed = 80}, false);
	
	changeIntakeState(false, false, true, false, false, false);
	pros::delay(1500);
	tongue.set_value(false);
	// back out of long goal
	chassis.moveToPoint(-45.5, 10, 1000);
	chassis.turnToHeading(180, 300, {}, false);
	doinker.set_value(false);
	chassis.moveToPoint(-45.5, 32.5, 3000, {.forwards=false,.minSpeed = 80}, false);
	chassis.turnToHeading(145, 1000);
}

void leftsideAlt(){
	chassis.setPose(-1, 0, 0);
	doinker.set_value(true);

	changeIntakeState(false, true, false, false, false, false);
	chassis.moveToPose( -10, 26, -21, 1500, {.minSpeed = 60});
	pros::delay(600);
	tongue.set_value(true);
	chassis.waitUntilDone();
	//chassis.moveToPose( -13.5, 31.5, -21, 500, {.minSpeed = 60}, false);
	pros::delay(200);
	chassis.turnToHeading(-125, 250, {}, false); // fix

	chassis.moveToPoint(-37, 3, 1500, {.maxSpeed = 90});
	pros::delay(200);

	chassis.turnToHeading(180, 250);
	tongue.set_value(true);
	// go into match load
	chassis.moveToPoint(-36, -20, 1450, {.maxSpeed = 50});
	chassis.moveToPoint(-36, 30, 1200, {.forwards=false,.maxSpeed = 80}, false);
	
	changeIntakeState(false, false, true, false, false, false);
	pros::delay(600);
	changeIntakeState(false, true, false, false, false, false);
	tongue.set_value(false);
	// back out of long goal
	chassis.moveToPoint(-36, 7, 1000, {}, false);
	changeIntakeState(false, false, false, false, false, true);
	chassis.moveToPose(9.5, 43.5, -131, 1000,{.forwards=false, .maxSpeed = 80}, false);
	pros::delay(3000);
	chassis.moveToPose(9.5, 43.5, -131, 1000, {.forwards=false, .minSpeed = 100}, false);
	changeIntakeState(false, false, false, true, false, false);
	pros::delay(1200);
	changeIntakeState(false, true, false, false, false, false);

	chassis.setPose(-10, 40, -135);
	// chassis.moveToPose(-45.5, 32.5, 0, 1500, {.minSpeed = 50, .earlyExitRange = 10}, false);
	// chassis.moveToPoint(-45.5, 36, 1500, {.maxSpeed = 90});
	// doinker.set_value(false);
	// chassis.turnToHeading(145, 1000);
	doinker.set_value(false);
	chassis.moveToPose(-32, 32, -90, 3000, {.minSpeed = 120, .earlyExitRange=3}, false);
	chassis.swingToHeading(0, DriveSide::RIGHT, 750, {.minSpeed = 90});
	// chassis.moveToPoint(-3, 15, 800, {.earlyExitRange = 0}, false);
	// //pros::delay(200);
	// chassis.turnToHeading(0, 1000, {}, false);
	// doinker.set_value(false);
	chassis.moveToPoint(-37, 39, 1000, {.minSpeed = 120});
	chassis.setPose(-37, 39, 0);
	chassis.moveToPoint(-35.5, 45, 500, {.minSpeed = 120});
	chassis.turnToHeading(45, 500);
}

void leftsideAlt2(){
	chassis.setPose(-1, 0, 0);
	doinker.set_value(true);

	changeIntakeState(false, true, false, false, false, false);
	chassis.moveToPose( -10, 26, -21, 1500, {.minSpeed = 60});
	pros::delay(600);
	tongue.set_value(true);
	chassis.waitUntilDone();
	//chassis.moveToPose( -13.5, 31.5, -21, 500, {.minSpeed = 60}, false);
	pros::delay(200);
	chassis.turnToHeading(-125, 250, {}, false); // fix

	chassis.moveToPoint(-37, 3, 1500, {.maxSpeed = 90});
	pros::delay(200);

	chassis.turnToHeading(180, 250);
	tongue.set_value(true);
	// go into match load
	chassis.moveToPoint(-36, -20, 1500, {.maxSpeed = 50});
	chassis.moveToPoint(-36, 30, 1200, {.forwards=false,.maxSpeed = 80}, false);
	
	changeIntakeState(false, false, true, false, false, false);
	pros::delay(700);
	changeIntakeState(false, true, false, false, false, false);
	tongue.set_value(false);
	// back out of long goal
	chassis.moveToPoint(-36, 7, 1000, {}, false);
	changeIntakeState(false, false, false, false, false, true);
	chassis.moveToPose(9.5, 43.5, -131, 1500,{.forwards=false, .minSpeed = 100}, false);
	changeIntakeState(false, false, false, true, false, false);
	pros::delay(1200);
	changeIntakeState(false, true, false, false, false, false);

	// mid goal high
	chassis.setPose(-10, 40, -135);
	// chassis.moveToPose(-45.5, 32.5, 0, 1500, {.minSpeed = 50, .earlyExitRange = 10}, false);
	// chassis.moveToPoint(-45.5, 36, 1500, {.maxSpeed = 90});
	// doinker.set_value(false);
	// chassis.turnToHeading(145, 1000);
	doinker.set_value(false);
	chassis.moveToPose(-32, 32, -90, 3000, {.minSpeed = 120, .earlyExitRange=3}, false);
	chassis.swingToHeading(0, DriveSide::RIGHT, 750, {.minSpeed = 90});
	// chassis.moveToPoint(-3, 15, 800, {.earlyExitRange = 0}, false);
	// //pros::delay(200);
	// chassis.turnToHeading(0, 1000, {}, false);
	// doinker.set_value(false);
	chassis.moveToPoint(-37, 39, 1000, {.minSpeed = 120});
	chassis.setPose(-37, 39, 0);
	chassis.moveToPoint(-35.5, 45, 500, {.minSpeed = 120});
	chassis.turnToHeading(45, 500);
}


void rightside7_motionchain(){
	// set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
	doinker.set_value(true);

	changeIntakeState(false, true, false, false, false, false);

	chassis.moveToPose(13, 32, 24, 750, {.minSpeed = 70, .earlyExitRange = 2.5}, true);
	pros::delay(500);

	tongue.set_value(true);
	//chassis.waitUntilDone();
	//pros::delay(200);

	chassis.swingToHeading(130, DriveSide::RIGHT, 60000, {.minSpeed = 90});

	//chassis.turnToHeading(-45, 600, {}, false);
	
	// chassis.moveToPoint(37, 6, 1000, {.earlyExitRange = 0});
	// chassis.turnToHeading(180,  300, {}, false); 

}

void rightside4plus3(){
	chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
	doinker.set_value(true);

	changeIntakeState(true, false, false, false, false, false);

	chassis.moveToPose(10, 28, 24, 2000, {.minSpeed = 60});
	pros::delay(500);
	tongue.set_value(true);
	chassis.waitUntilDone();
	pros::delay(200);
	chassis.turnToHeading(-45, 1000, {}, false);
	tongue.set_value(false);
	chassis.moveToPose(-9, 45, 131, 800,{.minSpeed = 60}, false);
	changeIntakeState(false, false, false, false, false, false);
	scoreSlow(false, true);
	pros::delay(900);
	scoreSlow(false, false);
	changeIntakeState(false, true, false, false, false, false);

	chassis.moveToPoint(36, 3, 1700, {.forwards = false}, false);
	tongue.set_value(true);
	pros::delay(200);
	chassis.turnToHeading(180,  1000, {}, false);

	
	chassis.moveToPose(34.5, -25, 180, 1500, {.maxSpeed = 50, .minSpeed = 50}, false);
	chassis.moveToPose(36, 30, 180,  1000, {.forwards = false, .minSpeed = 70}, false);
	changeIntakeState(false, false, true, false, false, false);
	pros::delay(1200);
	tongue.set_value(false);
	
	chassis.moveToPoint(27, 8, 1000);
	chassis.turnToHeading(180, 300, {}, false);
	doinker.set_value(false);
	chassis.moveToPoint(27, 30, 3000, {.forwards=false,.minSpeed = 60}, false);}


void rightside4plus3Alt(){
	// set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
	doinker.set_value(true);

	changeIntakeState(false, true, false, false, false, false);

	chassis.moveToPose(13, 32, 24, 2000, {.minSpeed = 60});
	pros::delay(500);
	tongue.set_value(true);
	chassis.waitUntilDone();
	pros::delay(200);
	chassis.turnToHeading(130, 600, {}, false);
	
	chassis.moveToPoint(37, 6, 1000, {.earlyExitRange = 1});
	chassis.turnToHeading(180,  500);

	chassis.moveToPose(37 ,-20, 180, 1600, {.maxSpeed = 50, .minSpeed = 50});
	tongue.set_value(true);
	chassis.moveToPose(37, 30, 180,  1000, {.forwards = false, .minSpeed = 70}, false);
	changeIntakeState(false, false, true, false, false, false);
	pros::delay(700);
	tongue.set_value(false);
	changeIntakeState(true, false, false, false, false, false);

	chassis.moveToPoint(37, 10, 1000, {}, false);
	pros::delay(200);

	chassis.turnToHeading(-45, 1000, {}, false);
	tongue.set_value(false);
	chassis.moveToPose(-7, 43, -45, 2000,{.minSpeed = 100});
	pros::delay(500);
	changeIntakeState(false, false, false, false, false, false);
	scoreSlow(false, true);
	pros::delay(2000);
	scoreSlow(false, false);
	changeIntakeState(false, true, false, false, false, false);

	chassis.moveToPoint(27.5, 15, 2000, {.forwards = false});
	pros::delay(300);
	chassis.turnToHeading(180, 600, {}, false);
	doinker.set_value(false);
	chassis.moveToPoint(27, 35, 3000, {.forwards=false,.minSpeed = 60}, false);
	chassis.turnToHeading(130, 1000);
}

void soloAWP(){
	chassis.setPose(-10, -2, 270);
	changeIntakeState(false, true, false, false, false, false);
	chassis.moveToPoint(-20, -2, 400, {.minSpeed = 60});
	chassis.moveToPoint(32, 3, 5000, {.forwards=false, .minSpeed = 60}, false);
	pros::delay(200);
	chassis.turnToHeading(180, 300);
	tongue.set_value(true);

	chassis.moveToPose(36, -25, 180, 1500, {.maxSpeed = 50, .minSpeed = 50});
	chassis.moveToPose(36, 30, 180,  900, {.forwards = false, .minSpeed = 100}, false);
	chassis.moveToPose(36, 50, 180,  800, {.forwards = false, .maxSpeed = 20});
	changeIntakeState(false, false, true, false, false, false);
	pros::delay(1000);
	tongue.set_value(false);

	chassis.turnToHeading(270, 700, {.maxSpeed=80});
	pros::delay(400);
	changeIntakeState(false, true, false, false, false, false);
	chassis.waitUntilDone();
	chassis.setPose(27, 20, chassis.getPose().theta);
	chassis.moveToPoint(0, 22, 1000, {.maxSpeed = 80, .earlyExitRange = 3});

	chassis.moveToPoint(-40, 22, 1400, {.maxSpeed = 100});
	pros::delay(700);
	tongue.set_value(true);
	chassis.turnToHeading(225, 300, {}, false);
	changeIntakeState(false, false, false, false, false, true);
	chassis.moveToPoint(-10, 61, 600, {.forwards=false}, false);
	changeIntakeState(false, false, false, false, false, false);
	scoreSlow(true, false);
	pros::delay(700);
	scoreSlow(false, false);
	changeIntakeState(false, true, false, false, false, false);

	chassis.moveToPoint(-66, 0, 1500, {.maxSpeed = 90});
	chassis.turnToHeading(180, 300);
	chassis.moveToPoint(-66, -20, 1500, {.maxSpeed = 50});
	chassis.moveToPose(-67, 40, 180, 800, {.forwards = false, .minSpeed = 80}, false);
	chassis.moveToPose(-67, 50, 180,  900, {.forwards = false, .maxSpeed = 40});
	changeIntakeState(false, false, true, false, false, false);
} 

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
		else if(scoreMidSlow){
			intakeRaise.set_value(false);
			intakeTop.move_velocity(600);
			intakeBottom.move_velocity(600);
			hood.set_value(false);
			midMech.set_value(true);
		}
		else if(scoreLowSlow){
			intakeRaise.set_value(true);
			intakeTop.move_velocity(-600);
			intakeBottom.move_velocity(-400);
		}
		else{
			intakeRaise.set_value(false);
			intakeTop.move_velocity(0);
			intakeBottom.move_velocity(0);
		}
		pros::delay(20);
	}
}

void autonomous(){
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	auton = true;
	pros::Task intakeAutonControl(intakeAutonController);
	// elim_left();
	// soloAWP();
	rightside7_motionchain(); // good
	//rightside4plus3();
	// rightside4plus3Alt();
	// leftside(); // good
	// leftsideAlt(); // good
	// leftsideAlt2();
	// switch (AutonSelector::getSelectedRoute()) {
	// 	case AutonRoute::RIGHT_SIDE:
	// 		rightside7();
	// 		break;
	// 	case AutonRoute::RIGHT_4PLUS3:
	// 		rightside4plus3();
	// 		break;
	// 	case AutonRoute::RIGHT_4PLUS3_ALT:
	// 		rightside4plus3Alt();
	// 		break;
	// 	case AutonRoute::LEFT_SIDE:
	// 		leftside();
	// 		break;
	// 	case AutonRoute::LEFT_ALT:
	// 		leftsideAlt();
	// 		break;
	// 	case AutonRoute::SOLO_AWP:
	// 		soloAWP();
	// 		break;
	// 	case AutonRoute::DO_NOTHING:
	// 		// Do nothing - robot stays still
	// 		break;
	// }
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
    double t = 10;
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
