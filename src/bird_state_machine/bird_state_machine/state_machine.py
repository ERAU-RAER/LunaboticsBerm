import rclpy
from rclpy.node import Node
from abc import ABC, abstractmethod

class StateMachine(Node):

    def __init__(self):
        from bird_state_machine.standby_state import StandbyState
        from bird_state_machine.setup_state import SetupState

        super().__init__('state_machine')

        self.state_list: State = []
        self.state_list.append(StandbyState(self))
        self.state_list.append(SetupState(self))

        self.current_state = self.state_list[0]
        self.current_state.enter()

    def change_state(self, old_state, new_state_name):
        if old_state == self.current_state:
            self.current_state.exit()
            self.current_state = self.get_state(new_state_name)
            self.current_state.enter()
    
    def get_state(self,state_name: str):
        for state in self.state_list:
            if state.state_name == state_name:
                return state
        return None

class State(ABC):
    
    @abstractmethod
    def __init__(self, state_machine: StateMachine, state_name: str):
        self.state_machine = state_machine
        self.state_name = state_name

    @abstractmethod 
    def enter(self):
        self.state_machine.get_logger().info('Entering %s' % self.state_name)
        
    @abstractmethod 
    def exit(self):
        self.state_machine.get_logger().info('Exiting %s' % self.state_name)
    

def main(args = None):
    rclpy.init(args=args)

    state_machine = StateMachine()
    rclpy.spin(state_machine)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
