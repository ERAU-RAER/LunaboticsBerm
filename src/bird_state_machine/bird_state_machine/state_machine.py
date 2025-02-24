import rclpy
from rclpy.node import Node
import std_msgs
from std_msgs.msg import String

class StateMachine(Node):

    def __init__(self):
        # Initialize ROS2 stuff
        super().__init__('state_machine')
        self.state_publisher_ = self.create_publisher(String,'state')

        # Import concrete states
        from bird_state_machine.state import State
        from bird_state_machine.standby_state import StandbyState
        from bird_state_machine.setup_state import SetupState

        # Initialize concrete states
        self.state_list: State = []
        self.state_list.append(StandbyState(self))
        self.state_list.append(SetupState(self))

        # Enter initial state
        self.current_state = self.state_list[0]
        self.current_state.enter()
        self.state_publisher_.publish(self.current_state.state_name)

    def change_state(self, old_state, new_state_name):
        if old_state is self.current_state and self.get_state(new_state_name) is not None:
            self.current_state.exit()
            self.current_state = self.get_state(new_state_name)
            self.current_state.enter()
            self.state_publisher_.publish(new_state_name)
    
    def get_state(self,state_name: str):
        for state in self.state_list:
            if state.state_name == state_name:
                return state
        return None

def main(args = None):
    rclpy.init(args=args)

    state_machine = StateMachine()
    rclpy.spin(state_machine)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
