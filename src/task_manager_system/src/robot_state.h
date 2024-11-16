#ifndef STATE_H
#define STATE_H

class RobotStateMachine;

class RobotState {
public:
    // Constructor and Destructor
    RobotState() = default;
    virtual ~RobotState() = default;

    // Virtual functions for state behavior
    virtual void enter(RobotStateMachine* robot) = 0;  // Called when entering the state
    virtual void loop(RobotStateMachine* robot) = 0;   // Called on each iteration of the main loop
    virtual void exit(RobotStateMachine* robot) = 0;   // Called when exiting the state
};

#endif // STATE_H