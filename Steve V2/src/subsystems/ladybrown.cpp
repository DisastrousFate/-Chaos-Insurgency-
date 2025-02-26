#include "main.h"
#include "headers/globals.h"
#include "headers/ladybrown.h"
#include "pros/rtos.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/timer.hpp"
#include <cstdlib>
#include "pros/screen.h"
#include "robodash/views/console.hpp"


using namespace Globals;

ladyBrown::ladyBrown() : ladybrownPID(2, 0, 0, 2, false) {
    ladybrown_encoder.reset();
    ladybrown_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}


void ladyBrown::nextState() {
    ladyBrown::currState += 1;
    if (ladyBrown::currState == 4) {
        ladyBrown::currState = 0;
    }
    ladyBrown::target = states[currState];
}

void ladyBrown::prevState() {
    ladyBrown::currState -= 1;
    if (ladyBrown::currState == -1) {
        ladyBrown::currState = 3;
    }
    ladyBrown::target = ladyBrown::states[ladyBrown::currState];
}

void ladyBrown::MoveToPoint(int target, int max_error, int timeout) {
    /*
    double kp = 2.2;
    double error = target - ladybrown_encoder.get_value();
    double velocity = kp * error;
    ladybrown_motor.move(velocity);
    */

    if(!isPIDrunning) {
        isPIDrunning = true;

        ladybrownPID.reset();
        lemlib::Timer timer(timeout);
        
        while (true) {
            double error = ladyBrown::target - ladybrown_encoder.get_value();
            double motor_voltage = ladybrownPID.update(error);

            if (std::abs(error) < max_error || timer.isDone()) {
                ladybrown_motor.brake();
                ladyBrown::isPIDrunning = false;
                break;
            }

            ladybrown_motor.move_voltage(motor_voltage);
            ladyBrown::isPIDrunning = true;
            pros::delay(20);
      }
        
    }
    
}

void ladyBrown::ladybrown_manual() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        ladybrown_motor.move(127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        ladybrown_motor.move(-127);
    } else {
        ladybrown_motor.move(0);
    }
}

void ladyBrown::ladybrown_run(bool async, int timeout){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        nextState();
    
    };
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        prevState();
    };

    if (!async) {
    MoveToPoint(ladyBrown::target);
    } else {
        int move_to = ladyBrown::target;
        pros::Task move([move_to, this]() { MoveToPoint(move_to); });
    }

};

