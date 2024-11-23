#pragma once
#include "robot_state.h"

class RobotStateMachine
{
public:
	RobotStateMachine();
	// Same as before
	inline RobotState* getCurrentState() const { return currentState; }
	
	// This will get called by the current state
	void setState(RobotState& newState);
	static RobotStateMachine& getInstance();

private:
	// LightState here is now a class, not the enum that we saw earlier
	RobotState* currentState;
};
