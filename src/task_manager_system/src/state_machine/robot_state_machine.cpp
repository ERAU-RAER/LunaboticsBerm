#include "robot_state_machine.h"
#include "standby_state.h"
#include "setup_state.h"

RobotStateMachine::RobotStateMachine()
{
  currentState = &StandbyState::getInstance();
}

void RobotStateMachine::setState(RobotState& newState)
{
	currentState->exit(this);  // do something before we change state
	currentState = &newState;  // change state
	currentState->enter(this); // do something after we change state
}

RobotStateMachine& RobotStateMachine::getInstance()
{
	static RobotStateMachine singleton;
	return singleton;
}


