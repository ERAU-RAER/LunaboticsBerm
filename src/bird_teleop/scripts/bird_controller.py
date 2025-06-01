#!/usr/bin/env python3
import rclpy

from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from std_srvs.srv import Trigger

class JoyToCmdVel:

    def __init__(self):

        self.node = rclpy.create_node('joy_to_cmdvel')

        self.max_speed = self.node.declare_parameter('max_speed', 0.25).value

        self.joy_sub = self.node.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.cmdvel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.zeropt_pub = self.node.create_publisher(Bool, 'zeropt', 10)
        self.cal_pub = self.node.create_publisher(Bool, 'arm_cal', 10)


    # Convert joy message into twist message
    def joy_callback(self, msg):
        twist_msg = Twist()
        
        deadband = 0.0125
        linear_input = msg.axes[1] if abs(msg.axes[1]) >= deadband else 0.0
        angular_input = msg.axes[0] if abs(msg.axes[0]) >= deadband else 0.0
        twist_msg.linear.x = linear_input * self.max_speed
        twist_msg.angular.z = angular_input * self.max_speed

        
        twist_msg.linear.z = msg.axes[5]
        twist_msg.angular.y = msg.axes[4]
        # Toggle zeropt using button index 1 (rising edge detection)
        # Initialize persistent state on first callback call if not already set
        if not hasattr(self, 'toggle_state'):
            self.toggle_state = False
            self.prev_button2 = 0

        # Check for rising edge on button index 2
        if msg.buttons[2] == 1 and self.prev_button2 == 0:
            self.toggle_state = not self.toggle_state
            toggle_msg = Bool()
            toggle_msg.data = self.toggle_state
            self.zeropt_pub.publish(toggle_msg)

        self.prev_button2 = msg.buttons[2]

        # cal_msg = Bool()
        # cal_msg.data = bool(msg.buttons[2])
        # self.cal_pub.publish(cal_msg)

        if msg.buttons[1]:
            # Call disable_motors service
            disable_client = self.node.create_client(Trigger, '/clearcore_driver_node/disable_motors')
            if not disable_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('Service /clearcore_driver_node/disable_motors does not exist.')
                return
            disable_request = Trigger.Request()
            disable_future = disable_client.call_async(disable_request)
            rclpy.spin_until_future_complete(self.node, disable_future)
            if disable_future.result() is not None:
                self.node.get_logger().info('Motors disabled')
            else:
                self.node.get_logger().error('Failed to call /clearcore_driver_node/disable_motors')
        
            # Call enable_motors service
            enable_client = self.node.create_client(Trigger, '/clearcore_driver_node/enable_motors')
            if not enable_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('Service /clearcore_driver_node/enable_motors does not exist.')
                return
            enable_request = Trigger.Request()
            enable_future = enable_client.call_async(enable_request)
            rclpy.spin_until_future_complete(self.node, enable_future)
            if enable_future.result() is not None:
                self.node.get_logger().info('Motors enabled')
            else:
                self.node.get_logger().error('Failed to call /clearcore_driver_node/enable_motors')

        # Only publish if the twist values have changed
        if not hasattr(self, 'last_twist'):
            self.last_twist = Twist()
            publish = True
        else:
            publish = (
            twist_msg.linear.x != self.last_twist.linear.x or
            twist_msg.angular.z != self.last_twist.angular.z or
            twist_msg.linear.z != self.last_twist.linear.z or
            twist_msg.angular.y != self.last_twist.angular.y
            )
        if publish:
            self.cmdvel_pub.publish(twist_msg)
            self.last_twist = twist_msg

def main(args=None):

    rclpy.init(args=args)

    joy_to_cmdvel = JoyToCmdVel()

    rclpy.spin(joy_to_cmdvel.node)

    joy_to_cmdvel.node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()



