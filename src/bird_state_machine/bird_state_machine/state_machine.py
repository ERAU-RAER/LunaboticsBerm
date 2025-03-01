import rclpy
from rclpy.node import Node
import std_msgs
from std_msgs.msg import String
from bird_interfaces.srv import ChangeState

class StateMachine(Node):

    def __init__(self):
        # Initialize ROS2 stuff
        super().__init__('state_machine')
        self.state_publisher_ = self.create_publisher(String,'state', 10)
        self.srv = self.create_service(ChangeState, 'change_state', self.change_state_callback)

        # Import states
        from bird_state_machine.state import State
        from bird_state_machine.standby_state import StandbyState
        from bird_state_machine.setup_state import SetupState

        # Initialize concrete states
        self.state_list: State = []
        self.state_list.append(StandbyState(self))
        self.state_list.append(SetupState(self))

        # Enter initial state
        self.current_state = self.state_list[0]
        self.current_state.enter(0)
        msg = String()
        msg.data = self.current_state.state_name
        self.state_publisher_.publish(msg)

    def change_state(self, old_state, new_state_name, error_code: int):
        if old_state is self.current_state and self.get_state(new_state_name) is not None:
            self.current_state.exit(error_code)
            self.current_state = self.get_state(new_state_name)
            self.current_state.enter(error_code)
            msg = String()
            msg.data = new_state_name
            self.state_publisher_.publish(msg)
    
    def get_state(self,state_name: str):
        for state in self.state_list:
            if state.state_name == state_name:
                return state
        return None
    
    def change_state_callback(self,request,response):
        self.change_state(self.current_state, request.new_state_name, request.error_code)
        response.success = True
        return response


def main(args = None):
    rclpy.init(args=args)

    state_machine = StateMachine()
    rclpy.spin(state_machine)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
