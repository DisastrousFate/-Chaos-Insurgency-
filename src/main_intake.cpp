#include "main.h"
#include "subsystems.hpp"

void mainintake_opcontrol()
{
    main_intake.set_brake_mode(MOTOR_BRAKE_HOLD); // set brake mode to hold

    // move motor at 100% speed when button L1 is pressed, on hoiding the button
    if (master.get_digital(DIGITAL_L1))
    {
      main_intake.move(127);  // Move the motor at full power while the button is held
    } else {
      main_intake.move(0);    // Stop the motor when the button is released
    }

    // reverse motor spin on L2
    if (master.get_digital(DIGITAL_L2))
    {
      main_intake.move(-127); // Move the motor at full power in reverse while the button is held
    } else {
      main_intake.move(0);    // Stop the motor when the button is released
    }
}