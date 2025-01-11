#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor main_intake(1);
void mainintake_opcontrol();

inline pros::Motor wall_stake(2);
void wallstake_opcontrol();

inline ez::Piston mogo_clamp_1('A', 0); // 0 == up, 1 == down
inline ez::Piston mogo_clamp_2('B', 0); // 0 == up, 1 == down
void mogoClamp_opcontrol();

inline ez::Piston front_intake('C', 0); // 0 == down, 1 == up
void fIntake_opcontrol();
