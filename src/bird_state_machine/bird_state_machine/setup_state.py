import rclpy
from rclpy.node import Node
from bird_state_machine.state_machine import StateMachine
from bird_state_machine.state import State


class SetupState(State):
    
    def __init__(self, state_machine: StateMachine):
        super().__init__(state_machine,'setup_state')      

    def enter(self):
        super().enter()
        pass

    def exit(self):
        super().exit()
        pass