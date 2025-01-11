#include "main.h"
#include "subsystems.hpp"

void mogoClamp_opcontrol()
{
    if (master.get_digital(DIGITAL_RIGHT))
    {
      printf("mogo_clamp_1 = %d\n", mogo_clamp_1.get());
      mogo_clamp_1.set(!mogo_clamp_1.get()); // toggle the piston

      printf("mogo_clamp_2 = %d\n", mogo_clamp_2.get());
      mogo_clamp_2.set(!mogo_clamp_2.get());
    }

    // alternative incase doesnt work

    //mogo_clamp_1.button_toggle(master.get_digital(DIGITAL_RIGHT));
    //mogo_clamp_2.button_toggle(master.get_digital(DIGITAL_LEFT));
}