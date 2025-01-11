#include "main.h"
#include "subsystems.hpp"

void fIntake_opcontrol()
{
    if (master.get_digital(DIGITAL_X))
    {
      printf("front_intake = %d\n", front_intake.get());
      front_intake.set(!front_intake.get()); // toggle the intake
    }
}