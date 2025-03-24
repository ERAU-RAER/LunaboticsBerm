import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import json
import threading

class g2siNode(Node):
    def __init__(self):
        super().__init__('g2si_node')

        # Publisher to send corrected IMU data
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10) 
        
        # Subscriber to receive raw IMU data
        self.subscription = self.create_subscription(Imu, '/imu/data_raw', self.listener_callback, 10)
        self.subscription  # Prevent unused variable warning

        # State variables
        self.data_received = False  # Tracks whether the first data sample has been received
        self.calibration_data = []  # Stores raw IMU data for calibration

        # Offsets for calibration (initialized to zero)
        self.accel_offsets = (0.0, 0.0, 0.0)
        self.gyro_offsets = (0.0, 0.0, 0.0)

        # Event to signal calibration completion
        self.calibration_event = threading.Event()

        # Timer to check if IMU data is received
        self.timer = self.create_timer(5.0, self.check_data_received)

        self.get_logger().info('Converting IMU linear acceleration from g\'s to m/s²')

    def listener_callback(self, msg):
        """
        Callback function to handle incoming IMU data.
        - Converts linear acceleration from g's to m/s².
        - Stores data for calibration if calibration is ongoing.
        - Publishes adjusted IMU data using the calculated offsets.
        """
        if not self.data_received:
            self.data_received = True
            self.get_logger().info('Found you! Received first IMU sample.')

        # During calibration, store raw IMU data for processing
        if not self.calibration_event.is_set():
            raw_data = json.dumps({
                'ax': msg.linear_acceleration.x,
                'ay': msg.linear_acceleration.y,
                'az': msg.linear_acceleration.z,
                'gx': msg.angular_velocity.x,
                'gy': msg.angular_velocity.y,
                'gz': msg.angular_velocity.z
            })
            self.calibration_data.append(raw_data)

        # Convert linear acceleration to m/s² using calibration offsets
        converted_acceleration = self.convert_g_to_ms2(msg.linear_acceleration)

        # Prepare and publish a new IMU message with adjusted values
        new_imu_msg = Imu()
        new_imu_msg.header = msg.header
        new_imu_msg.orientation = msg.orientation
        new_imu_msg.angular_velocity = msg.angular_velocity
        new_imu_msg.linear_acceleration = converted_acceleration

        self.publisher_.publish(new_imu_msg)

    def convert_g_to_ms2(self, accel_in_gs: Vector3) -> Vector3:
        """
        Converts acceleration from g's to m/s² and applies calibration offsets.
        :param accel_in_gs: Acceleration in g's (x, y, z components)
        :return: Adjusted acceleration in m/s²
        """
        conversion_factor = 9.81  # 1 g = 9.81 m/s²
        accel_in_ms2 = Vector3()

        # Apply calibration offsets and convert to m/s²
        accel_in_ms2.x = (accel_in_gs.x - self.accel_offsets[0]) * conversion_factor
        accel_in_ms2.y = (accel_in_gs.y - self.accel_offsets[1]) * conversion_factor
        accel_in_ms2.z = (accel_in_gs.z - self.accel_offsets[2]) * conversion_factor

        return accel_in_ms2

    def hl_calibrate_imu(self, num_samples=1000):
        """
        Performs IMU calibration by collecting data samples, calculating offsets, 
        and applying them for future adjustments.
        :param num_samples: Number of samples to collect for calibration.
        """
        self.get_logger().info("Starting IMU calibration... Keep sensor flat and steady! This is gonna take a hot second...")

        ax = ay = az = gx = gy = gz = 0.0

        # Collect enough data samples for calibration
        while len(self.calibration_data) < num_samples:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Process collected data to calculate offsets
        for i in range(num_samples):
            try:
                data = self.calibration_data[i]
                json_data = json.loads(data)

                # Sum up raw IMU values for averaging
                ax += float(json_data.get('ax', 0.0))
                ay += float(json_data.get('ay', 0.0))
                az += float(json_data.get('az', 0.0))
                gx += float(json_data.get('gx', 0.0))
                gy += float(json_data.get('gy', 0.0))
                gz += float(json_data.get('gz', 0.0))

            except Exception as e:
                self.get_logger().warning(f"Error during calibration data collection: {e}")
                continue

        # Calculate average offsets
        accel_offset_x = ax / num_samples
        accel_offset_y = ay / num_samples
        accel_offset_z = (az / num_samples) - 8192  # Adjust for Earth's gravity (~1g)

        gyro_offset_x = gx / num_samples
        gyro_offset_y = gy / num_samples
        gyro_offset_z = gz / num_samples

        # Store calculated offsets
        self.accel_offsets = (accel_offset_x, accel_offset_y, accel_offset_z)
        self.gyro_offsets = (gyro_offset_x, gyro_offset_y, gyro_offset_z)

        self.get_logger().info("IMU Calibration complete!")
        self.get_logger().info(f"Accel Offsets: x={accel_offset_x}, y={accel_offset_y}, z={accel_offset_z}")
        self.get_logger().info(f"Gyro Offsets: x={gyro_offset_x}, y={gyro_offset_y}, z={gyro_offset_z}")

        # Signal calibration completion
        self.calibration_event.set()

    def check_data_received(self):
        """
        Checks if any IMU data has been received. If not, it logs a warning.
        """
        if not self.data_received:
            self.get_logger().warn('Hello? Is anyone out there? Still waiting on first IMU sample...')

def main(args=None):
    rclpy.init(args=args)
    node = g2siNode()

    # Optional: Perform calibration at startup
    node.hl_calibrate_imu()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
