#include "robot/drivetrain.h"
#include "api.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

int Drivetrain::CheckDeadzone(int ControllerInput) {
    if(std::abs(ControllerInput) < Drivetrain::deadzone) {
        return 0;
    }
    else {
        return ControllerInput;
    }
}

Drivetrain::Drivetrain() {
    Drivetrain::deadzone = 5;
}

// Drive functions
void Drivetrain::TankDrive(){
    // get left y and right y positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // move the robot
    chassis.tank(leftY, rightY);
}

void Drivetrain::ArcadeDrive(){
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

    // move the robot
    chassis.arcade(leftY, leftX);
}

void Drivetrain::Doublestick_Arcade(){
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    chassis.arcade(leftY, rightX);
}

void Drivetrain::CurvatureDrive(){
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

    // move the robot
    chassis.curvature(leftY, leftX);
}

void Drivetrain::Double_CurvatureDrive(){
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    chassis.curvature(leftY, rightX);
}
