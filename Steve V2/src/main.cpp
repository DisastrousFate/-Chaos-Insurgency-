#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "globals.h"
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
#include "robot/ladybrown.h"

using namespace Robot;
using namespace Robot::Globals;

/*

       ████████▒░▒█████████ ▒░░▒██▒░░▒ ██ ▒░░████████    ████████   
    ░▒▓█▓▒░░▒▓█▓▒░     ░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░      ░▒▓█▓▒░░▒ █▓▒░ 
    ░▒▓█▓▒░░▒▓█▓▒░     ░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░      ░▒▓█▓▒░░▒ █▓▒░ 
     ░▒▓██████▓▒░▒▓███████▓▒░░▒▓████████▓▒░▒▓███████▓▒░░▒▓█▓▒░░▒ █▓▒░ 
    ░▒▓█▓▒░░▒▓█▓▒░     ░▒▓█▓▒░      ░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒ █▓▒░ 
    ░▒▓█▓▒░░▒▓█▓▒░     ░▒▓█▓▒░      ░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒ █▓▒░ 
     ░▒▓██████▓▒░▒▓███████▓▒░       ░▒▓█▓▒░░▒▓██████▓▒░░▒▓███████▓▒░  

    _______  _______  _______  _______  _______  _______  _______  _______ 

    _____ _____ _____ _____ _____    _____ _____ _____ _____ _____ _____ _____ _____ _____ __ __ 
    |     |  |  |  _  |     |   __|  |     |   | |   __|  |  | __  |   __|   __|   | |     |  |  |
    |   --|     |     |  |  |__   |  |-   -| | | |__   |  |  |    -|  |  |   __| | | |   --|_   _|
    |_____|__|__|__|__|_____|_____|  |_____|_|___|_____|_____|__|__|_____|_____|_|___|_____| |_|  
                                                                                              
    _______  _______  _______  _______  _______  _______  _______  _______  
     
    ** 8346D ** 
    ** -- Chaos Insurgency -- **
    ** Written and Developed by Alexander Halesworth **

        \\Special Thanks to the LEM Library and PROS Contributors\\
        \\Special Thanks to the RoboDash Team\\
                                                                                                                                                                                                                                                      
*/

/**
 * @file main.cpp
 * @brief This file contains the main code for the robot's operation.
*/

namespace Robot {
/**
 * @brief Structure that holds instances of all robot subsystems.
 */
struct RobotSubsystems {
	//Robot::Autonomous autonomous;
	Robot::Drivetrain drivetrain;
	Robot::Clamp clamp;
    Robot::Doinker doinker;
    Robot::Intake intake;
    Robot::LadyBrown ladybrown;
	//Robot::Elevation elevation;
	//Robot::Puncher puncher;
	//Robot::Intake intake;
} subsystem;
}

ASSET(lbq1_txt);
ASSET(lbq2_txt);

// Check if the A button is pressed to toggle the mogo clamp
bool doinker_up = false;

static int intakeProcess_time = 500; // ms
static int intakeCapture_time = 1000; // ms
static int ladybrown_speed = 127;


static int wallarm_speed = 60;
int wallarm_angle = 0;

static int mogoDelay_time = 200; // ms
static int autonTimeout = 10000; // ms

void getWallPos(){
    wallarm_angle = ladybrown_motor.get_position();
}

// Auton Functions
/*
void A_intakeRing(){
    intake_motor.move(ladybrown_speed);
    pros::delay(intakeProcess_time);
    intake_motor.move(0);
}

void A_captureRing(){
    intake_motor.move(-ladybrown_speed);
    pros::delay(intakeCapture_time);
    intake_motor.move(0);
}

void A_spinIntake(){
    intake_motor.move(-ladybrown_speed);
}

void A_stopIntake(){
    intake_motor.move(0);
}

void A_mogoClamp(){
    mogo_engaged = !mogo_engaged;

    mogo_pistons.toggle();
    pros::delay(mogoDelay_time);
}

void A_doinker(){
    doinker_up = !doinker_up;

    doinker.toggle();
}

// Auton Routines

void Skills(){
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(-47.462, 17.854, 180);
    pros::delay(mogoDelay_time);

    A_mogoClamp();
    pros::delay(mogoDelay_time);
    A_spinIntake();
} 

void newSkills(){ // robot garden gambit/st chris/letchworth
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

void left_redQual(){
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

//BREAD AUTON
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
    pros::delay(1000);
    chassis.turnToHeading(270, 1000);
    pros::delay(1000);
    chassis.moveToPose(15, 15, 180, 1000);
    pros::delay(1000);
    // go back to starting point
    //chassis.moveToPoint(0, 0, 10000);
    pros::delay(1000);
    //chassis.turnToHeading(0, 1000);
}

/*


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
    
    console.println("   ████████▒░▒█████████ ▒░░██▒░░▒ ██ ▒░░  ████████   ████████");
    console.println("░▒▓█▓▒░░▒▓█▓▒░     ░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░      ░▒▓█▓▒░░▒ █▓▒░");
    console.println("░▒▓█▓▒░░▒▓█▓▒░     ░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░      ░▒▓█▓▒░░▒ █▓▒░ ");
    console.println(" ░▒▓██████▓▒░▒▓███████▓▒░░▒▓████████▓▒░▒▓███████▓▒░░▒▓█▓▒░░▒ █▓▒░ ");
    console.println("░▒▓█▓▒░░▒▓█▓▒░     ░▒▓█▓▒░      ░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒ █▓▒░ ");
    console.println("░▒▓█▓▒░░▒▓█▓▒░     ░▒▓█▓▒░      ░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒ █▓▒░");
    console.println(" ░▒▓██████▓▒░▒▓███████▓▒░       ░▒▓█▓▒░░▒▓██████▓▒░░▒▓███████▓▒░  ");
    console.println("_______  _______  _______  _______  _______  _______  _______  _______");
    console.println(" _____ _____ _____ _____ _____    _____ _____ _____ _____ _____ _____ _____ _____ _____ __ __ ");
    console.println("|     |  |  |  _  |     |   __|  |     |   | |   __|  |  | __  |   __|   __|   | |     |  |  |");
    console.println("|   --|     |     |  |  |__   |  |-   -| | | |__   |  |  |    -|  |  |   __| | | |   --|_   _|");
    console.println("|_____|__|__|__|__|_____|_____|  |_____|_|___|_____|_____|__|__|_____|_____|_|___|_____| |_|  ");
    console.println("_______  _______  _______  _______  _______  _______  _______  _______ ");
                                                                    

	pros::delay(1000); 
	chassis.calibrate(); // calibrate sensors
    chassis.setPose(0,0,0);

    //motor configs
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   /// ladybrown_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    ladybrown_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); 
    wallarm_angle = ladybrown_motor.get_position();
    console.printf("Wall Arm Position: %d\n", wallarm_angle);


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
    /*
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
    */
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

    //selector.run_auton();
    printf("Running auton...");
    
}


void wallarm_ready(){

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
		
        //doublestick_arcade();

        //console.printf("Wall Arm Position: %d\n", ladybrown_motor.get_position());

        // select autonomous and run autonomous without a competition switch
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			competition_initialize();
		}
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			autonomous();
		}

        subsystem.drivetrain.run();

        //----------------------//
        //      Pneumatics      //
        //----------------------//
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            subsystem.clamp.toggle();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
           subsystem.doinker.toggle();
        }

        //----------------------//
        //        Intake        //
        //----------------------//

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){

            subsystem.intake.moveForward();

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            subsystem.intake.moveBackward();

        } else {

            intake_motor.brake();
        }

        //-----------------------------------//
        //      Wall Arm / Lady Brown        //
        //-----------------------------------//
        /*
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            subsystem.ladybrown.Raise();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            subsystem.ladybrown.Lower();
        } else {
            ladybrown_motor.brake();
        }*/
        subsystem.ladybrown.run();

        printf("Wall Arm Position: %d\n", ladybrown_encoder.get_value());
        


        // delay to save resources
        pros::delay(25);
	}
}