#include "robot/doinker.h"
#include "api.h"
#include "globals.h"

using namespace Robot::Globals;

void Robot::Doinker::toggle(){
    printf("Doinker Toggled");
    console.println("Doinker Toggled");
    engaged = doinker.is_extended();

    doinker.toggle();
}

