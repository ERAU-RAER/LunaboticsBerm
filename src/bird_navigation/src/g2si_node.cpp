#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>

using namespace std::chrono_literals;

class G2siNode : public rclcpp::Node
{
public:
    G2siNode()
    : Node("g2si_node"),
      data_received_(false),
      calibration_complete_(false)
    {
        this->declare_parameter<bool>("use_cal", true);

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", 10,
            std::bind(&G2siNode::listenerCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            5s, std::bind(&G2siNode::checkDataReceived, this));

        RCLCPP_INFO(this->get_logger(), "Converting IMU linear acceleration from g's to m/s²");

        // ===== START calibration in background =====
        std::thread(&G2siNode::hlCalibrateIMU, this).detach();
    }

    void hlCalibrateIMU(size_t num_samples = 500)
    {
        bool use_cal;
        this->get_parameter("use_cal", use_cal);

        if (!use_cal)
        {
            RCLCPP_INFO(this->get_logger(), "Calibration disabled via parameter, skipping.");
            calibration_complete_ = true;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting IMU calibration... keep sensor flat & steady!");

        {
            std::unique_lock<std::mutex> lock(calibration_mutex_);
            if (!calibration_cv_.wait_for(lock, 120s, [this, num_samples]() {
                    return calibration_data_.size() >= num_samples;
                }))
            {
                RCLCPP_WARN(this->get_logger(), "Calibration timed out (got %zu of %zu samples).",
                            calibration_data_.size(), num_samples);
                calibration_complete_ = true;   // <— don’t block forever
                return;
            }
        }

        // compute offsets
        double ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
        for (auto &d : calibration_data_)
        {
            ax += d[0]; ay += d[1]; az += d[2];
            gx += d[3]; gy += d[4]; gz += d[5];
        }
        size_t n = calibration_data_.size();
        accel_offsets_ = { ax/n, ay/n, (az/n)-1.0 };
        gyro_offsets_  = { gx/n, gy/n, gz/n };

        RCLCPP_INFO(this->get_logger(), "Calibration complete! Accel offsets (g): x=%.3f y=%.3f z=%.3f",
                    accel_offsets_[0], accel_offsets_[1], accel_offsets_[2]);
        RCLCPP_INFO(this->get_logger(), "Gyro offsets (rad/s): x=%.3f y=%.3f z=%.3f",
                    gyro_offsets_[0], gyro_offsets_[1], gyro_offsets_[2]);

        calibration_complete_ = true;
    }

private:
    void listenerCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!data_received_)
        {
            data_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First IMU sample received.");
        }

        std::array<double,6> sample = {
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z,
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        };

        // still calibrating? keep gathering and bail out
        if (!calibration_complete_)
        {
            {
                std::lock_guard<std::mutex> lock(calibration_mutex_);
                calibration_data_.push_back(sample);
                if (calibration_data_.size() % 50 == 0)
                    RCLCPP_INFO(this->get_logger(), "Collected %zu calibration samples",
                                calibration_data_.size());
            }
            calibration_cv_.notify_one();
            return;
        }

        // post‐calibration: sliding window
        moving_window_.push_back(sample);
        if (moving_window_.size() > max_window_size_)
            moving_window_.pop_front();

        // compute average
        std::array<double,6> avg = {0,0,0,0,0,0};
        for (auto &w : moving_window_)
            for (size_t i=0; i<6; ++i)
                avg[i] += w[i];
        for (size_t i=0; i<6; ++i)
            avg[i] /= moving_window_.size();

        geometry_msgs::msg::Vector3 acc_avg, gyr_avg;
        acc_avg.x = avg[0]; acc_avg.y = avg[1]; acc_avg.z = avg[2];
        gyr_avg.x = avg[3]; gyr_avg.y = avg[4]; gyr_avg.z = avg[5];

        // convert g→m/s²
        auto conv_acc = convertGToMs2(acc_avg);

        // publish filtered + calibrated IMU
        sensor_msgs::msg::Imu out{};
        out.header = msg->header;
        out.orientation = msg->orientation;
        out.angular_velocity = gyr_avg;
        out.linear_acceleration = conv_acc;
        publisher_->publish(out);
    }

    geometry_msgs::msg::Vector3 convertGToMs2(const geometry_msgs::msg::Vector3 &in)
    {
        constexpr double G = 9.81;
        geometry_msgs::msg::Vector3 out;
        out.x = (in.x - accel_offsets_[0]) * G;
        out.y = (in.y - accel_offsets_[1]) * G;
        out.z = (in.z - accel_offsets_[2]) * G;
        return out;
    }

    void checkDataReceived()
    {
        if (!data_received_)
            RCLCPP_WARN(this->get_logger(), "Waiting for IMU data...");
    }

    // ROS interfaces
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // state
    bool data_received_;
    bool calibration_complete_;
    std::vector<std::array<double,6>> calibration_data_;
    std::mutex calibration_mutex_;
    std::condition_variable calibration_cv_;

    std::array<double,3> accel_offsets_{0,0,0}, gyro_offsets_{0,0,0};

    // moving‐window
    std::deque<std::array<double,6>> moving_window_;
    const size_t max_window_size_ = 10;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<G2siNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
