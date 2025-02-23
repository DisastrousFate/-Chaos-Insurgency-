#include "robot/clamp.h"
#include "api.h"
#include "globals.h"

using namespace Robot::Globals;

void Robot::Clamp::toggle(){
    printf("Mogo Toggled");
    console.println("Mogo Toggled");
    engaged = mogo_pistons.is_extended();

    mogo_pistons.toggle();
}

