#include "main.h"
#include "headers/globals.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include "headers/ladybrown.h"

namespace Globals {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    pros::Motor ladybrown_motor(18);
    pros::adi::Encoder ladybrown_encoder(
                pros::adi::ext_adi_port_tuple_t(19, 'G', 'H'),
                true);

}