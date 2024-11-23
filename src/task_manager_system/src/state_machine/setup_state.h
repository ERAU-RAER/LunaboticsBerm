#pragma once
#include "robot_state_machine.h"
#include "robot_state.h"

class SetupState : public RobotState
{
public:
    // Virtual functions for state behavior
    void enter(RobotStateMachine* robot) {}  // Called when entering the state
    void loop(RobotStateMachine* robot) {}   // Called on each iteration of the main loop
    void exit(RobotStateMachine* robot) {}   // Called when exiting the state
    void emergency_exit(RobotStateMachine* robot) {}  // Called when the Fault Detection System forces a state change
    
    static RobotState& getInstance();

private:
    SetupState() {};
    SetupState(const SetupState& other);
    SetupState& operator=(const SetupState& other);
};