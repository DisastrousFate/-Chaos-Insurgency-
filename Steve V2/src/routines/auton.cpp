#include "robot/auton.h"
#include "api.h"
#include "globals.h"
#include "robot/intake.h"

using namespace Robot;
using namespace Robot::Globals;


//namespace Robot {
/**
 * @brief Structure that holds instances of all robot subsystems.
 */


// Autonomous Functions

/*
void intakeRing(){
    subsystem.intake.moveForward();
    pros::delay(intakeProcess_time);
    intake_motor.brake();
}

void captureRing(){
    subsystem.intake.moveBackward();
    pros::delay(intakeCapture_time);
    intake_motor.brake();
}

void spinIntake(){
    intake_motor.move(-ladybrown_speed);
}

void stopIntake(){
    intake_motor.brake();
}

void mogoClamp(){
    subsystem.clamp.toggle();
}

void doinker(){
    subsystem.doinker.toggle();
}
*/