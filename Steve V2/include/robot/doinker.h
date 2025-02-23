#pragma once


namespace Robot {
/**
 * @brief The Clamp class represents the MoGo clamp of a robot.
 *
 * The Clamp class provides functionality to control and operate the MoGo clamp of a robot.
 * It allows the robot to extend and retract the position of the clamp, while keeping track of its state.
 */
class Doinker {
public:
    /**
     * @brief Toggles the Mogo Clamp.
     *
     * This function extends/retracts the clamp
     *
     * @note Make sure to initialize the clamp before calling this function.
     */

    Doinker();

    void toggle();


private:
    /** The current state of the clamp. 
     *  The state variable represents the current state of the clamp.
     *  It can be either true or false, indicating whether the clamp is activated or deactivated.
     *  This variable is used to control the behavior of the clamp and to query their current state.
     */
    bool engaged;
};

}