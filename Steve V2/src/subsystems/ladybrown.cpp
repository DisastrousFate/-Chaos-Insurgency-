#include "robot/ladybrown.h"
#include "api.h"
#include "globals.h"

using namespace Robot::Globals;


void Robot::LadyBrown::Raise(){
    intake_motor.move(intake_speed);
}

void Robot::LadyBrown::Lower(){
    intake_motor.move(-intake_speed);
}

void Robot::LadyBrown::stop(){
    intake_motor.move(0);
}