#pragma once
#include <string>

class RobotStateMachine;

class RobotState {

public:
    // Virtual functions for state behavior
    virtual void enter(RobotStateMachine* robot) = 0;  // Called when entering the state
    virtual void loop(RobotStateMachine* robot) = 0;   // Called on each iteration of the main loop
    virtual void exit(RobotStateMachine* robot) = 0;   // Called when exiting the state
    virtual void emergency_exit(RobotStateMachine* robot) = 0;  // Called when the Fault Detection System forces a state change
    
    virtual ~RobotState() {}
};