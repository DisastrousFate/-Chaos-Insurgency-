#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include "robodash/api.h"
#include "robodash/views/selector.hpp"

ASSET(example_txt);
ASSET(auton_skills_v1_txt);


pros::Controller controller(pros::E_CONTROLLER_MASTER);


// Pneumatics

//pros::adi::DigitalOut mogo1('A', 0); // assuming 'A' is the port for the piston
//pros::adi::DigitalOut mogo2('B',0); // assuming 'B' is the port for the piston

pros::adi::Pneumatics mogo_piston1('A', true, true);
pros::adi::Pneumatics mogo_piston2('B', true, true);

pros::adi::DigitalOut fintake('C', 0); // assuming 'A' is the port for the piston

// Motors

pros::Motor intake_motor(8);
pros::Motor wall_arm(2);


//intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

// Check if the A button is pressed to toggle the mogo clamp
static bool mogo_engaged = false;
static bool fintake_up = false;

pros::MotorGroup left_motors(
	{7, 6},
	pros::MotorGearset::green
);

pros::MotorGroup right_motors(
	{-5, -4},
	pros::MotorGearset::green
);

lemlib::Drivetrain drivetrain(
	&left_motors,   
	&right_motors,
	13,
	lemlib::Omniwheel::OLD_4,
	200,
	2
);

// Uncomment when tracking wheels are added!

/*
// parallel/vertical encoder
pros::adi::Encoder vertical_adi_encoder('E', 'F'); // add true parameter to reverse

// perpendicular/horizontal encoder
pros::adi::Encoder horizontal_adi_encoder('G', 'H'); // add true parameter to reverse

//vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_adi_encoder, lemlib::Omniwheel::NEW_275, -2.5);

//horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_adi_encoder, lemlib::Omniwheel::NEW_275, -5.75);







lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
*/

pros::Imu imu(11); // inertial sensor

// placeholder/temp setup with plain imu odom

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void Skills(){
    chassis.setPose(0, 0, 0);
    chassis.follow(auton_skills_v1_txt, 15, 2000); // edit values here
}

void example(){
    chassis.setPose(0, 0, 0);
    chassis.follow(example_txt, 15, 2000); // edit values here
}

// PID Tuning
void tunePID(){
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
}

rd::Selector selector({
    {"Skills run V1", &Skills},
    {"example", &example},
    {"PID Tuning", &tunePID},
});

rd::Console console; /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	//pros::lcd::initialize();

    console.clear();
    console.println("Robodash is running");
	//pros::lcd::set_text(1, "Hello Alexander!");

	/* Run to check optical shaft encoder inversion

	while (true) { // infinite loop
        // print measurements from the adi encoder

        pros::lcd::print(0, "ADI Encoder: %i", adi_encoder.get_value());
        pros::delay(10); // delay to save resources. DO NOT REMOVE
    }
	*/
	pros::delay(1000); 
	chassis.calibrate(); // calibrate sensors

    //motor configs
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wall_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


    

	// print position to brain screen
    /*pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });*/
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
void competition_initialize() {}

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


void autonomous() {

    /*pros::Task calibrateTask([=]() {
        chassis.calibrate();
        
    });*/
    pros::delay(1000);
    /*
    pros::Task telemetryTask([&]() {
        console.println("Running LemLib auton");
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            pose = chassis.getPose();
            console.printf("X: %f Y:%f Theta: %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
            printf("X: %f, Y: %f, Theta: %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
            // delay to save resources
            pros::delay(1000);
        }
    });
    */

    selector.run_auton();
    printf("Running auton...");
    
}

// Drive functions
void tankdrive()
{
    // get left y and right y positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // move the robot
    chassis.tank(leftY, rightY);
}



void arcadedrive()
{
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

    // move the robot
    chassis.arcade(leftY, leftX);
}

void doublestick_arcade(){
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    chassis.arcade(leftY, rightX);
}

void curvaturedrive(){
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

    // move the robot
    chassis.curvature(leftY, leftX);
}

void double_curvaturedrive(){
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    chassis.curvature(leftY, rightX);
}


void toggle_mogo() {

    printf("Engaged");
    printf(mogo_piston1.is_extended() ? "true" : "false");
    console.println("Engaged");
    console.println(mogo_piston1.is_extended() ? "true" : "false");
    mogo_engaged = !mogo_engaged;

    mogo_piston1.toggle();
    mogo_piston2.toggle();

    /* ALTERNATE SETUP MANUAL TOGGLE
        if (mogo_piston1.is_extended()){
            mogo_piston1.retract();
            mogo_piston2.retract();
        } else {
            mogo_piston1.extend();
            mogo_piston2.extend();
        }
    */

    // OLD CODE
    //mogo1.set_value(mogo_engaged);
    //mogo2.set_value(mogo_engaged);

}

void toggle_fintake(){
    fintake_up = !fintake_up;
    fintake.set_value(fintake_up);
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

void opcontrol() {
	
	while (true) {
		
        doublestick_arcade();

        //----------------------//
        //      Pneumatics      //
        //----------------------//
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            toggle_mogo();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            toggle_fintake();
        }

        //----------------------//
        //        Intake        //
        //----------------------//
   
         // move motor at 100% speed when button L1 is pressed, on hoiding the button

        /*
        if (controller.get_digital(DIGITAL_L1))
        {
            intake_motor.move(-127);  // Move the motor at full power while the button is held
        } else {
            intake_motor.move(0);    // Stop the motor when the button is released
        }
        
        // reverse motor spin on L2
        if (controller.get_digital(DIGITAL_L2))
        {
            intake_motor.move(127); // Move the motor at full power in reverse while the button is held
            //intake_motor.move_velocity(const std::int32_t velocity)
        } else {
            intake_motor.move(0);    // Stop the motor when the button is released
        }
        */

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){

            intake_motor.move(-100);

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {

            intake_motor.move(100);
        } else {

            intake_motor.move(0);
        }

        /*
        if (controller.get_digital(DIGITAL_DOWN))
        {
            wall_arm.move(127);
        } else {
            wall_arm.move(0);
        }

        if (controller.get_digital(DIGITAL_B))
        {
            wall_arm.move(-127);
        } else {
            wall_arm.move(0);
        }
        */

        //----------------------//
        //      Wall Arm        //
        //----------------------//

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            wall_arm.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            wall_arm.move(-127);
        } else {
            wall_arm.move(0);
        }


        // delay to save resources
        pros::delay(25);
	}
}