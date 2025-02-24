import rclpy
from rclpy.node import Node
from bird_state_machine.state_machine import StateMachine
from bird_state_machine.state import State


class StandbyState(State):
    
    def __init__(self, state_machine: StateMachine):
        super().__init__(state_machine,'standby_state')

    def enter(self):
        super().enter()
        self.state_machine.change_state(self,'setup_state')

    def exit(self):
        super().exit()
        pass