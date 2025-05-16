import rclpy

from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class JoyToCmdVel:

    def __init__(self):

        self.node = rclpy.create_node('joy_to_cmdvel')

        self.max_speed = self.node.declare_parameter('max_speed', 0.1).value

        self.joy_sub = self.node.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.cmdvel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.zeropt_pub = self.node.create_publisher(Bool, 'zeropt', 10)


    # Convert joy message into twist message
    def joy_callback(self, msg):
        twist_msg = Twist()
        twist_msg.linear.x = msg.axes[3] * self.max_speed
        twist_msg.angular.z = msg.axes[2] * self.max_speed
        twist_msg.linear.z = msg.axes[5]
        twist_msg.angular.y = msg.axes[4]
        self.cmdvel_pub.publish(twist_msg)
        
        # Print indices of currently pressed buttons
        pressed_buttons = [i for i, button in enumerate(msg.buttons) if button]
        self.node.get_logger().info("Pressed buttons: {}".format(pressed_buttons))

def main(args=None):

    rclpy.init(args=args)

    joy_to_cmdvel = JoyToCmdVel()

    rclpy.spin(joy_to_cmdvel.node)

    joy_to_cmdvel.node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()



