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
#include <cstdio>
#include "robodash/api.h"
#include "robodash/views/selector.hpp"
#include "pros/adi.h"
#include "headers/ladybrown.h"
#include "headers/globals.h"

using namespace Globals;


struct Subsystems {

    ladyBrown lb;

} subsystem;

ASSET(lbq1_txt);
ASSET(lbq2_txt);

<<<<<<< HEAD
ASSET(skills1_txt);


pros::Controller controller(pros::E_CONTROLLER_MASTER);


=======
>>>>>>> 4aa7691b13b61ba26a08f3a740d10c56ab7f2873
// Pneumatics
pros::adi::Pneumatics mogo_piston1('A', true, true);
pros::adi::Pneumatics plonker('B', true, true);
pros::adi::Ultrasonic ultrasonic('G', 'H'); // FIX, input, output

double allianceStake_distance = 3.5; // centimeters

// Motors
pros::Motor intake_motor(-12);

//intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

// Check if the A button is pressed to toggle the mogo clamp
static bool mogo_engaged = false;
static bool plonker_up = false;

static int intakeProcess_time = 500; // ms
static int intakeCapture_time = 1000; // ms
static int intake_speed = 127;

static int mogoDelay_time = 200; // ms
static int autonTimeout = 10000; // ms

pros::MotorGroup left_motors(
	{-13, -8, 9}, //9
	pros::MotorGearset::blue
);

pros::MotorGroup right_motors(
	{14, 5, -4}, //5
	pros::MotorGearset::blue
);

lemlib::Drivetrain drivetrain(
	&left_motors,   
	&right_motors,
	13,
	lemlib::Omniwheel::OLD_4,
	420,
	2
);  

//parallel/vertical encoder
pros::adi::Encoder vertical_adi_encoder('C', 'D', true); // add true parameter to reverse

// perpendicular/horizontal encoder
pros::adi::Encoder horizontal_adi_encoder('E', 'F', true); // add true parameter to reverse

//vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_adi_encoder, lemlib::Omniwheel::OLD_275_HALF, 0.5);

//horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_adi_encoder, lemlib::Omniwheel::OLD_275_HALF, -4.5);

pros::Imu imu(11); // inertial sensor

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
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
/*
lemlib::ControllerSettings lateral_controller(13, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);*/

// angular PID controller
lemlib::ControllerSettings angular_controller(4.3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              27.3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);



// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// Auton Functions


void A_intakeRing(){
    intake_motor.move(intake_speed);
    pros::delay(intakeProcess_time);
    intake_motor.move(0);
}

void A_captureRing(){
    intake_motor.move(-intake_speed);
    pros::delay(intakeCapture_time);
    intake_motor.move(0);
}

void A_spinIntake(){
    intake_motor.move(-intake_speed);
}

void A_stopIntake(){
    intake_motor.move(0);
}

void A_mogoClamp(){
    mogo_engaged = !mogo_engaged;

    mogo_piston1.toggle();
    pros::delay(mogoDelay_time);
}

void A_plonker(){
    plonker_up = !plonker_up;

    plonker.toggle();
}

float allianceStake_allign(){
    lemlib::Pose pose = chassis.getPose();
    float poseX = pose.x;
    float poseY = pose.y;
    float distance = ultrasonic.get_value();
    float difference = distance - allianceStake_distance;
    if (difference  > 0){ // too far
        if(chassis.getPose().x < 0){ // left side of field
            chassis.moveToPoint(poseX - difference, poseY, autonTimeout, {}, false);
        } else {
            chassis.moveToPoint(poseX + difference, poseY, autonTimeout, {}, false);
        }
    } else {
        if(chassis.getPose().x > 0){ // right side of field
            chassis.moveToPoint(poseX + difference, poseY, autonTimeout, {}, false);
        } else {
            chassis.moveToPoint(poseX - difference, poseY, autonTimeout, {}, false);
        }
    }
    
    

}

// Auton Routines

void Skills(){
    chassis.setPose(-58.726, -0.312, 270);
    allianceStake_allign();

    // print pose
    printf("X: %f, Y: %f, Theta: %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

    chassis.moveToPose(-47.657, -0.312, 0, autonTimeout, {}, false);
    chassis.moveToPoint(-47.114, -23.462, autonTimeout, {}, false);
    A_mogoClamp();

    A_spinIntake();
    chassis.turnToHeading(70, autonTimeout);
    chassis.follow(skills1_txt, 10, 80000, true, false);
    


} 

void newSkills(){ // robot garden gambit
    A_spinIntake();
    pros::delay(1500);
    A_stopIntake();

    chassis.setPose(-60.867, 0.396, 90);
    chassis.moveToPoint(-46.956, 0.407, autonTimeout, {}, false);

    chassis.turnToHeading(0, 1000, {}, false);
    chassis.moveToPoint(-47.491, -22.868,  autonTimeout, {}, false);
    A_mogoClamp();
    A_spinIntake();
    chassis.turnToHeading(90, autonTimeout, {}, false);
    chassis.moveToPoint(-23.146, -23.938, autonTimeout, {}, false);
    chassis.turnToHeading(180, autonTimeout, {}, false);
    chassis.moveToPoint(-23.681 , -62.729, autonTimeout, {}, false);
    chassis.turnToHeading(90, autonTimeout, {}, false);
    chassis.moveToPoint(-60.064, -64.601,autonTimeout, {}, false);
    A_stopIntake();
    A_mogoClamp();

}


void Qual(){
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0,30,2000);
    pros::delay(mogoDelay_time);
    //chassis.moveToPose(10,40, 90, 4000);

}

void alliance(){
    chassis.setPose(0,0,0);
    A_spinIntake();
    pros::delay(5000);
    A_stopIntake();

    chassis.moveToPoint(0, 30, 2000, {}, false);
    pros::delay(mogoDelay_time);
}

void red_Qual2(){
    chassis.setPose(-150, 60, 180, 1000);
    chassis.moveToPose(-46, 60, 180, 4000);
    A_mogoClamp();
    pros::delay(1000);
    A_spinIntake();
    chassis.moveToPose(36, 130, 90, 5000);
}

void blue_Qual2(){
    chassis.setPose(150, 60, 0, 1000);
    chassis.moveToPoint(60,60,4000);
    A_mogoClamp();
    A_spinIntake();
    chassis.moveToPoint(60,160,3500);
}

void left_redQual(){ // Left Red Qualifications / Left Right Qualifications.
    chassis.setPose(-58.19, 23.288, 270);
    chassis.follow(lbq1_txt, 10, autonTimeout, false, false);
    A_mogoClamp();
    A_spinIntake();

    chassis.turnToHeading(0, 1000, {}, false);
    chassis.moveToPoint(-23.574, 47.162, autonTimeout, {}, false);
    pros::delay(1000);
    A_mogoClamp();
    
    chassis.turnToHeading(180,1000, {}, false);

    chassis.setPose(-23.574, 47.223, 180);
    chassis.follow(lbq2_txt, 10, autonTimeout, true, false);


}


//BREAD AUTOON
void BlueRightAuton () {
    chassis.setPose(0,0,0);
    chassis.moveToPose(0,-15,0,1500,{.forwards = false, .maxSpeed = 60}, true );
    chassis.moveToPose (0,-29,0,1500, {.forwards = false, .maxSpeed = 30}, true);
    pros::delay(4500);
    A_mogoClamp();
    pros::delay(1000);
    A_spinIntake();
    pros::delay(2000);
    chassis.moveToPose(-24,-24,315,3000,{.forwards = true},true);
    chassis.moveToPose(-24,-15,45,2000,{.forwards=true},true);
}

void RedLeftAuton(){
    chassis.setPose(0,0,0);
    chassis.moveToPose(0,-29,0,4000,{.forwards = false, .maxSpeed = 30}, false);
    A_mogoClamp();
    pros::delay(1000);
    A_spinIntake();
}

// PID Tuning
void tunePID(){
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    //chassis.turnToHeading(90, 100000);
    chassis.moveToPoint(0, 20, 1000);
    /*pros::delay(1000);
    chassis.turnToHeading(270, 1000);
    pros::delay(1000);
    chassis.moveToPose(15, 15, 180, 1000);
    pros::delay(1000);
    // go back to starting point
    //chassis.moveToPoint(0, 0, 10000);
    pros::delay(1000);
    //chassis.turnToHeading(0, 1000);*/
}



rd::Console console; /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
rd::Selector selector({
        {"Qual", &Qual},
        {"PID Tuning", &tunePID},
        {"Alliance", &alliance},
        {"leftRedQual", &left_redQual},
        {"RedLeftAuton", &RedLeftAuton},
        {"BlueRightAuton", &BlueRightAuton},
        {"autoskills", &newSkills}
        //{"pathBlueQual", &pathBlueQual}

    });

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
    console.println("updated");
	//pros::lcd::set_text(1, "Hello Alexander!");

	
	
	pros::delay(1000); 
	chassis.calibrate(); // calibrate sensors

    //motor configs
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   // console.printf("Wall Arm Position: %d\n", ladybr);



/* Run to check optical shaft encoder inversion

	while (true) { // infinite loop
        // print measurements from the adi encoder

        printf("VERT Encoder: %i\n", vertical_adi_encoder.get_value());
        printf("HORZ Encoder: %i\n", horizontal_adi_encoder.get_value());

        //pros::lcd::print(0, "ADI Encoder: %i", vertical_adi_encoder.get_value());
        //console.printf("VERT Encoder: %i\n", vertical_adi_encoder.get_value());
        //console.printf("HORZ Encoder: %i\n", horizontal_adi_encoder.get_value());
        pros::delay(25); // delay to save resources. DO NOT REMOVE
    }
    
*/

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

/*
void tunePIDValue(double &value, const char* name) {
    console.printf("Tuning %s: %f\n", name, value);
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            value += 0.5;
            console.printf("%s increased to: %f\n", name, value);
            chassis.setAngularControllerSettings(angular_controller); // Apply changes to the angular controller
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            value -= 0.5;
            console.printf("%s decreased to: %f\n", name, value);
            chassis.setAngularControllerSettings(angular_controller); // Apply changes to the angular controller
        }
        pros::delay(100);
    }
}
*/

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


void autonomous() {

    /*pros::Task calibrateTask([=]() {
        chassis.calibrate();
        
    });*/
    //pros::delay(1000);
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
void tankdrive(){
    // get left y and right y positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // move the robot
    chassis.tank(leftY, rightY);
}

void arcadedrive(){
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

    printf("Mogo Toggled");
    console.println("Mogo Toggled");
    mogo_engaged = !mogo_engaged;

    mogo_piston1.toggle();

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

void toggle_plonker(){
    plonker_up = !plonker_up;
    console.println("plonker toggled");
    printf("plonker toggled");

    plonker.toggle();
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
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            toggle_mogo();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            toggle_plonker();
        }

        //----------------------//
        //        Intake        //
        //----------------------//

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){

            intake_motor.move(intake_speed);

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {

            intake_motor.move(-intake_speed);
        } else {

            intake_motor.move(0);
        }

<<<<<<< HEAD
        
       /* if (controller.get_digital(DIGITAL_DOWN))
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


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
			nextState();
		}
=======
        subsystem.lb.ladybrown_run();
>>>>>>> 4aa7691b13b61ba26a08f3a740d10c56ab7f2873

        // delay to save resources
        pros::delay(25);
	}
}