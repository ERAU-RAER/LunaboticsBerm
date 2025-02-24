from bird_state_machine.state_machine import StateMachine
from abc import ABC, abstractmethod

class State(ABC):
    
    @abstractmethod
    def __init__(self, state_machine: StateMachine, state_name: str):
        self.state_machine = state_machine
        self.state_name = state_name

    @abstractmethod 
    def enter(self):
        self.state_machine.get_logger().info('State Machine: Entering %s' % self.state_name)
        
    @abstractmethod 
    def exit(self):
        self.state_machine.get_logger().info('State Machine: Exiting %s' % self.state_name)   
