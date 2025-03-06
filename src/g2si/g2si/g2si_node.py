import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class g2siNode(Node):
    def __init__(self):
        super().__init__('g2si_node')

        # Setup
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10) # Publisher to publish the new IMU data with converted acceleration in m/s²
        self.subscription = self.create_subscription(Imu, '/imu/data_raw',self.listener_callback,10) # Subscriber to the IMU topic
        self.subscription  # prevent unused variable warning

        self.data_received = False
        self.timer = self.create_timer(5.0, self.check_data_received)

        self.get_logger().info(f'Converting IMU linear acceleration from g\'s to m/s²')

    def listener_callback(self, msg):
        if not self.data_received:
            self.data_received = True
            self.get_logger().info('Found you! Received first IMU sample')

        # Convert linear acceleration from g's to m/s^2
        converted_acceleration = self.convert_g_to_ms2(msg.linear_acceleration)

        new_imu_msg = Imu()
        new_imu_msg.header = msg.header
        new_imu_msg.orientation = msg.orientation
        new_imu_msg.angular_velocity = msg.angular_velocity

        # Replace the linear acceleration with the converted data
        new_imu_msg.linear_acceleration = converted_acceleration

        # Publish the new IMU message
        self.publisher_.publish(new_imu_msg)

    def convert_g_to_ms2(self, accel_in_gs: Vector3) -> Vector3:
        """
        Converts the acceleration from g's to m/s^2.
        :input accel_in_gs: Acceleration in g's (x, y, z components)
        :return: Converted acceleration in m/s^2
        """
        conversion_factor = 9.81  # 1 g = 9.81 m/s^2
        accel_in_ms2 = Vector3()
        accel_in_ms2.x = accel_in_gs.x * conversion_factor
        accel_in_ms2.y = accel_in_gs.y * conversion_factor
        accel_in_ms2.z = accel_in_gs.z * conversion_factor
        return accel_in_ms2

    def check_data_received(self):
        if not self.data_received:
            self.get_logger().warn('Hello? Is anyone out there? Still waiting on first IMU sample.')

def main(args=None):
    rclpy.init(args=args)
    node = g2siNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()