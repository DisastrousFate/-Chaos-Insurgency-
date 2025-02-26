#pragma once

#include "lemlib/pid.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "headers//globals.h"



class ladyBrown {
public:

    ladyBrown();
    void ladybrown_manual();
    void ladybrown_run(bool async = true, int timeout = 1000);

    void MoveToPoint(int target, int max_error = 150, int timeout = 1000);

private:

    int numOfStates = 4;
    int states[4] = {0, 24, 130, 180};
    int currState;
    int target;

    bool isPIDrunning;

    lemlib::PID ladybrownPID;



    void nextState();
    void prevState();
    void liftControl();
};