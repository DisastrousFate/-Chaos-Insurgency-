#include "robot/intake.h"
#include "api.h"
#include "globals.h"

using namespace Robot::Globals;


void Robot::Intake::moveForward(){
    intake_motor.move(intake_speed);
}

void Robot::Intake::moveBackward(){
    intake_motor.move(-intake_speed);
}

void Robot::Intake::stop(){
    intake_motor.move(0);
}