#pragma once
#include "api.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "robodash/api.h"
#include "robodash/views/selector.hpp"


/**
 * @file globals.h
 * @brief Contains global variables and type definitions for the Robot namespace.
 */

/**
 * @namespace Robot
 * @details In order to construct the robot, the Robot namespace is used to contain all of the objects that are used to control the robot. This includes the subsystems, methods, and global objects.
 * We use PROS extensively for the robot, it can be found at <A HREF="https://pros.cs.purdue.edu">PROS</A>
*/

namespace Robot {

    /**
     * @brief Contains global variables and type definitions for the Robot namespace.
     * @details The majority of the global variables are defined in the Globals namespace. This is to allow for easy access to the variables from any file in the project. 
     * The Globals namespace is also used to hold lemlib objects that are used to control the autonomous functions of the robot. It contains parameters that are used to control
     * the PID that lemlib uses, additionally.
     */
    namespace Globals {
        extern pros::Controller controller;

        extern pros::adi::Pneumatics mogo_pistons;
        extern pros::adi::Pneumatics doinker;

        extern pros::Motor intake_motor;
        extern pros::Motor ladybrown_motor;

        extern bool mogo_engaged;
        extern bool doinker_up;

        extern int intakeProcess_time;
        extern int intakeCapture_time;
        extern int intake_speed;

        extern int ladybrown_speed;
        extern int ladybrown_angle;

        extern int mogoDelay_time;
        extern int autonTimeout;

        extern pros::MotorGroup left_motors;
        extern pros::MotorGroup right_motors;
        extern lemlib::Drivetrain drivetrain;

        extern pros::adi::Encoder vertical_adi_encoder;
        extern pros::adi::Encoder horizontal_adi_encoder;
        extern lemlib::TrackingWheel vertical_tracking_wheel;
        extern lemlib::TrackingWheel horizontal_tracking_wheel;

        extern pros::Imu imu;

        extern lemlib::OdomSensors sensors;
        extern lemlib::ControllerSettings lateral_controller;
        extern lemlib::ControllerSettings angular_controller;

        extern lemlib::Chassis chassis;
        extern rd::Console console;
    }
} // namespace Robot
