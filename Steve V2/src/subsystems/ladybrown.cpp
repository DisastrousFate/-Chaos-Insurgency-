#include "robot/ladybrown.h"
#include "api.h"
#include "globals.h"
#include <iostream>
#include "lemlib/pid.hpp"
#include "lemlib/timer.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cstdlib>

using namespace Robot;
using namespace Robot::Globals;

LadyBrown::LADYBROWN_STATE LadyBrown::current_state = LadyBrown::BASE_STATE;


LadyBrown::LadyBrown() : MoveToPointPID(2, 0, 0, 2, false) {
    ladybrown_motor.set_zero_position(0);
    ladybrown_encoder.get_value();
    ladybrown_encoder.reset();
    
   ///ladybrown_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   ///ladybrown_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
void LadyBrown::run(bool async, int timeout) {
   LADYBROWN_STATE move_to;

   if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      if (current_state == BASE_STATE) {
         move_to = LOAD_STATE;
      } else if (current_state == LOAD_STATE) {
         move_to = ATTACK_STATE;
      } else {
         move_to = BASE_STATE;
      }
    printf("Moving to: %d\n", move_to);
    printf("current state: %d\n", current_state);

      if (!async) {
         MoveToPoint(move_to);
      } else {

         pros::Task move([move_to, this]() { MoveToPoint(move_to); });
      }

      if (!isPIDRunning) {
         current_state = move_to;
      }

   } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {

      if (current_state == ATTACK_STATE) {
         move_to = LOAD_STATE;
      } else if (current_state == LOAD_STATE) {
         move_to = BASE_STATE;
      } else {
         return;
      }

      std::cout << "Moving to: " << move_to << std::endl;
      std::cout << "current state: " << current_state << std::endl;

      if (!async) {
         MoveToPoint(move_to);
      } else {
         pros::Task move([move_to, this]() { MoveToPoint(move_to); }, "LadyBrownMove");
      }

      if (!isPIDRunning) {
         current_state = move_to;
      }
   }
}

int LadyBrown::get_target() { return target; }

void LadyBrown::MoveToPoint(LADYBROWN_STATE state, int max_error, int timeout) {

   std::cout << "state: " << current_state << std::endl;
   constexpr double base_location = 0;
   constexpr double load_location = 30;
   constexpr double attack_location = 120;

   int target;

   std::cout << "state: " << state << std::endl;
   std::cout << "pid: " << isPIDRunning << std::endl;

   if (!isPIDRunning) {

      std::cout << "inner pid: " << isPIDRunning << std::endl;
      LadyBrown::isPIDRunning = true;

      switch (state) {
      case LADYBROWN_STATE::BASE_STATE:
         target = base_location;
         break;
      case LADYBROWN_STATE::LOAD_STATE:
         target = load_location;
         break;
      case LADYBROWN_STATE::ATTACK_STATE:
         target = attack_location;
         break;
      }

      std::cout << "target: " << target << std::endl;

      MoveToPointPID.reset();

      lemlib::Timer timer(timeout);

      while (true) {
         //double error = target - ladybrown_motor.get_position();
         double error = target - ladybrown_encoder.get_value();
         double motor_voltage = MoveToPointPID.update(error);

         // motor_voltage = lemlib::slew(motor_voltage, LadyBrownMotor.get_voltage(), 1500);

         if (std::abs(error) < max_error || timer.isDone()) {
            ladybrown_motor.brake();
            LadyBrown::isPIDRunning = false;
            LadyBrown::current_state = state;
            break;
         }

         ladybrown_motor.move_voltage(motor_voltage);
         LadyBrown::isPIDRunning = true;
         pros::delay(20);
      }
   }
}


void Robot::LadyBrown::Raise(){
    ladybrown_motor.move(60);
}

void Robot::LadyBrown::Lower(){
    ladybrown_motor.move(-60);
}

void Robot::LadyBrown::stop(){
    ladybrown_motor.brake();
}