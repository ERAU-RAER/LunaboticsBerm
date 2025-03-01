import rclpy
from rclpy.node import Node
from bird_state_machine.state_machine import StateMachine
from bird_state_machine.state import State


class StandbyState(State):
    
    def __init__(self, state_machine: StateMachine):
        super().__init__(state_machine,'standby_state')

    def enter(self, error_code: int):
        super().enter(error_code)

    def exit(self, error_code: int):
        super().exit(error_code)
        pass