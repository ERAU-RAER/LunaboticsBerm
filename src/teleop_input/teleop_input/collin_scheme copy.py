import rclpy

from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist


class JoyToCmdVel:
    

    def __init__(self):

        self.max_speed = .1
        self.old_bumper = 0
        self.old_vibe_button = 0
        self.vibe = 0
        
        self.node = rclpy.create_node('joy_to_cmdvel')

        self.joy_sub = self.node.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.cmdvel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)


    # Convert joy message into twist message
    def joy_callback(self, msg):

        if msg.buttons[4] == 1 and self.old_bumper == 0 and self.max_speed > .7:
            self.max_speed -= .1
            self.old_bumper = 1
        elif msg.buttons[5] == 1 and self.old_bumper == 0 and self.max_speed < 1:
            self.max_speed += .1
            self.old_bumper = 1
        elif msg.buttons[4] == 0 and msg.buttons[5] == 0 and self.old_bumper == 1:
            self.old_bumper = 0

        twist_msg = Twist()

        twist_msg.linear.x = round(-self.max_speed*((msg.axes[5]-1) - (msg.axes[2]-1))/2,3) # forward/Backward
        twist_msg.angular.z = -(msg.axes[3]) # left/right
        twist_msg.linear.z = msg.axes[7] # bucket vertical travel
        twist_msg.angular.y = float(msg.axes[6]) # bucket rotation
        
        #vibrate bucket
        if msg.buttons[1] == 1 and self.old_vibe_button == 0:
            self.vibe = not self.vibe
            twist_msg.angular.x = float(self.vibe)
            self.old_vibe_button = 1
        elif msg.buttons[1] == 0 and self.old_vibe_button == 1:
            self.old_vibe_button = 0
        


def controller(args=None):

    rclpy.init(args=args)

    joy_to_cmdvel = JoyToCmdVel()

    rclpy.spin(joy_to_cmdvel.node)

    joy_to_cmdvel2.node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()



