#include "globals.h"
#include "api.h"
#include "robot/drivetrain.h"

/*
* Although the following constants belong in their own seperate files(auton.cpp, drivetriain.cpp), they are put here in order to maintain 
* a common location for all of the constants used by the program to belong in.
* NOTE: This is the location where these variables are put into memory, but they can be otherwise modified throughout the program.
*/


// Defines the objects that are used by the program for each of the individual subsystems.

namespace Robot {
    namespace Globals {

        pros::Controller controller (pros::E_CONTROLLER_MASTER);

        /* COPIED CODE
        pros::Motor RightFront (6, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
        pros::Motor LeftFront (-5, pros:: E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
        pros::Motor LeftBack (-7, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
        pros::Motor RightBack (8, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
        pros::Motor LeftMid (-2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
        pros::Motor PuncherMotor (19, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
        pros::Motor PuncherMotor2 (-20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
        pros::Motor RightMid (3, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
        pros::Motor IntakeMotor (-9, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
        */

        bool mogo_engaged = false;
        bool doinker_down = true;

        pros::adi::Pneumatics mogo_pistons('A', true, true);
        pros::adi::Pneumatics doinker('B', true, true);

        pros::Motor intake_motor(-12);
        pros::Motor ladybrown_motor(18);

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

        pros::Imu imu(9); // inertial sensor

        lemlib::OdomSensors sensors(
            &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
            &horizontal_tracking_wheel, // horizontal tracking wheel 1
            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
            &imu // inertial sensor
        );

        // lateral PID controller
        lemlib::ControllerSettings lateral_controller(
            10, // proportional gain (kP)
            0, // integral gain (kI)
            3, // derivative gain (kD)
            3, // anti windup
            1, // small error range, in inches
            100, // small error range timeout, in milliseconds
            3, // large error range, in inches
            500, // large error range timeout, in milliseconds
            20 // maximum acceleration (slew)
        );

        lemlib::ControllerSettings angular_controller(
            4.3, // proportional gain (kP)
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
        lemlib::Chassis chassis(
            drivetrain, // drivetrain settings
            lateral_controller, // lateral PID settings
            angular_controller, // angular PID settings
            sensors // odometry sensors
        );

        /* COPIED CODE
        pros::ADIDigitalOut FrontWing ('A');
        pros::ADIDigitalOut BackWing1 ('B');
        pros::ADIDigitalOut BackWing2 ('C');
        pros::ADIDigitalOut Elevator ('D');

        pros::ADIDigitalIn puncherToggleSwitch('E');
        pros::ADIDigitalIn autonToggleSwitch('F');
        pros::ADIDigitalIn drivetrainToggleSwitch('G');

        pros::Motor_Group punchers ({PuncherMotor, PuncherMotor2});
        pros::Motor_Group drive_left ({LeftFront, LeftMid, LeftBack});
        pros::Motor_Group drive_right ({RightFront, RightMid, RightBack});
        pros::Motor_Group drive_ ({LeftFront, RightFront, LeftMid, RightMid, LeftBack, RightBack});
        */
    }
}