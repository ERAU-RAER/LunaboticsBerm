#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>

using namespace std::chrono_literals;

class G2siNode : public rclcpp::Node
{
public:
    G2siNode() : Node("g2si_node"), data_received_(false), calibration_complete_(false)
    {
        // Declare the use_cal parameter with a default value of true
        this->declare_parameter<bool>("use_cal", true);

        // Publisher to send corrected IMU data
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // Subscriber to receive raw IMU data
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", 10,
            std::bind(&G2siNode::listenerCallback, this, std::placeholders::_1));

        // Timer to check if IMU data is received
        timer_ = this->create_wall_timer(
            5s, std::bind(&G2siNode::checkDataReceived, this));

        RCLCPP_INFO(this->get_logger(), "Converting IMU linear acceleration from g's to m/s²");
    }

    void hlCalibrateIMU(size_t num_samples = 500)
    {
        // Check if calibration is enabled via the use_cal parameter
        bool use_cal;
        this->get_parameter("use_cal", use_cal);

        if (!use_cal)
        {
            RCLCPP_INFO(this->get_logger(), "Calibration is disabled via parameter. Skipping calibration.");
            calibration_complete_ = true; // Mark as complete to avoid reattempts
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting IMU calibration... Keep sensor flat and steady!");

        // Wait until enough samples are collected
        {
            std::unique_lock<std::mutex> lock(calibration_mutex_);
            if (!calibration_cv_.wait_for(lock, 120s, [this, num_samples]() {
                    return calibration_data_.size() >= num_samples;
                }))
            {
                RCLCPP_WARN(this->get_logger(), "Calibration timed out. Not enough samples collected.");
                return;
            }
        }

        // Initialize accumulators
        double ax = 0.0, ay = 0.0, az = 0.0, gx = 0.0, gy = 0.0, gz = 0.0;

        // Process collected data to calculate offsets
        for (const auto &data : calibration_data_)
        {
            ax += data[0];
            ay += data[1];
            az += data[2];
            gx += data[3];
            gy += data[4];
            gz += data[5];
        }

        // Calculate average offsets
        size_t actual_samples = calibration_data_.size();
        accel_offsets_ = {
            ax / actual_samples,
            ay / actual_samples,
            (az / actual_samples) - 1.0
        };


        gyro_offsets_ = {
            gx / num_samples,
            gy / num_samples,
            gz / num_samples
        };

        RCLCPP_INFO(this->get_logger(), "IMU Calibration complete!");
        RCLCPP_INFO(this->get_logger(), "Accel Offsets: x=%.3f, y=%.3f, z=%.3f",
                    accel_offsets_[0], accel_offsets_[1], accel_offsets_[2]);
        RCLCPP_INFO(this->get_logger(), "Gyro Offsets: x=%.3f, y=%.3f, z=%.3f",
                    gyro_offsets_[0], gyro_offsets_[1], gyro_offsets_[2]);

        calibration_complete_ = true;
    }

private:
    void listenerCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!data_received_)
        {
            data_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Found you! Received first IMU sample.");
        }
    
        // During calibration, store raw IMU data
        if (!calibration_complete_)
        {
            {
                std::lock_guard<std::mutex> lock(calibration_mutex_);
                calibration_data_.emplace_back(std::array<double, 6>{
                    msg->linear_acceleration.x,
                    msg->linear_acceleration.y,
                    msg->linear_acceleration.z,
                    msg->angular_velocity.x,
                    msg->angular_velocity.y,
                    msg->angular_velocity.z});
            }
            calibration_cv_.notify_one(); // Notify waiting thread
        }
    
        // Convert linear acceleration to m/s² using calibration offsets
        geometry_msgs::msg::Vector3 converted_acceleration = convertGToMs2(msg->linear_acceleration);
    
        // Prepare and publish a new IMU message with adjusted values
        auto new_imu_msg = sensor_msgs::msg::Imu();
        new_imu_msg.header = msg->header;
        new_imu_msg.orientation = msg->orientation;
        new_imu_msg.angular_velocity = msg->angular_velocity;
        new_imu_msg.linear_acceleration = converted_acceleration;
    
        publisher_->publish(new_imu_msg);
    }

    geometry_msgs::msg::Vector3 convertGToMs2(const geometry_msgs::msg::Vector3 &accel_in_gs)
    {
        constexpr double conversion_factor = 9.81; // 1 g = 9.81 m/s²
        geometry_msgs::msg::Vector3 accel_in_ms2;

        accel_in_ms2.x = (accel_in_gs.x - accel_offsets_[0]) * conversion_factor;
        accel_in_ms2.y = (accel_in_gs.y - accel_offsets_[1]) * conversion_factor;
        accel_in_ms2.z = (accel_in_gs.z - accel_offsets_[2]) * conversion_factor;

        return accel_in_ms2;
    }

    void checkDataReceived()
    {
        if (!data_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Hello? Is anyone out there? Still waiting on first IMU sample...");
        }
    }

    // ROS 2 components
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    bool data_received_;
    bool calibration_complete_;
    std::vector<std::array<double, 6>> calibration_data_;
    std::mutex calibration_mutex_;
    std::condition_variable calibration_cv_; // Added condition variable

    // Calibration offsets
    std::array<double, 3> accel_offsets_{0.0, 0.0, 0.0};
    std::array<double, 3> gyro_offsets_{0.0, 0.0, 0.0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<G2siNode>();

    // Optional: Perform calibration at startup
    // node->hlCalibrateIMU();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}