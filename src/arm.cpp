#include "main.h"
#include "subsystems.hpp"

// -------------------------
// Wall stakes / Hanging arm
// -------------------------

void wallstake_opcontrol()
{
    wall_stake.set_brake_mode(MOTOR_BRAKE_HOLD); // set brake mode to hold

    // move motor at 100% speed when button R1 is pressed, on hoiding the button
    if (master.get_digital(DIGITAL_R1))
    {
        wall_stake.move(127);  // Move the motor at full power while the button is held
    } else {
        wall_stake.move(0);    // Stop the motor when the button is released
    }

    // reverse motor spin on R2
    if (master.get_digital(DIGITAL_R2))
    {
        wall_stake.move(-127); // Move the motor at full power in reverse while the button is held
    } else {
        wall_stake.move(0);    // Stop the motor when the button is released
    }
}
