#pragma once
#include "lemlib/pid.hpp"

namespace Robot {

    /**
     * @brief The Intake class represents a robot intake system.
     */
    class LadyBrown {
    public:
        /**
         * @brief Runs the main function of the intake system.
         * 
         * This function takes user input to control the intake system.
         */

        enum LADYBROWN_STATE {
            BASE_STATE = 1,
            LOAD_STATE = 2,
            ATTACK_STATE = 3
        };

         void run(bool async = true, int timeout = 1000);

        void MoveToPoint(LadyBrown::LADYBROWN_STATE state, int max_error = 150, int timeout = 1000);

        LadyBrown();

        int get_target();

        void Raise();
        void Lower();
        void stop();

    private:
        lemlib::PID MoveToPointPID;
        static LADYBROWN_STATE current_state;
        int target;
        bool isPIDRunning;
    };
}