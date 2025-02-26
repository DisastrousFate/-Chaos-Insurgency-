#pragma once

#include "main.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

namespace Globals {

    extern pros::Controller controller;
    extern pros::Motor ladybrown_motor;
    extern pros::adi::Encoder ladybrown_encoder;

}
