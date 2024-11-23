#include "standby_state.h"

void enter(RobotStateMachine* robot)  // Called when entering the state
{    

} 

void loop(RobotStateMachine* robot) // Called on each iteration of the main loop
{

}  

void exit(RobotStateMachine* robot) // Called when exiting the state
{

}  

void emergency_exit(RobotStateMachine* robot)   // Called when the Fault Detection System forces a state change
{

}

RobotState& StandbyState::getInstance()
{
	static StandbyState singleton;
	return singleton;
}