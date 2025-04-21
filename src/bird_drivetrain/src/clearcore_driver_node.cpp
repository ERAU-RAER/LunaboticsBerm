#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>
#include <chrono>
#include <string>
#include <numbers>

using namespace std::chrono_literals;

class ClearcoreDriverNode : public rclcpp::Node
{
public:
    ClearcoreDriverNode() : Node("clearcore_driver_node") {
        // parameters
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->get_parameter("port", port_);
        this->get_parameter("baudrate", baudrate_);

        // open serial
        try {
            serial_.setPort(port_);
            serial_.setBaudrate(baudrate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(to);
            serial_.open();
            RCLCPP_INFO(get_logger(), "Opened serial %s @ %d", port_.c_str(), baudrate_);
        } catch (std::exception &e) {
            RCLCPP_FATAL(get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // setup
        setVerboseFeedback(true); // enable verbose feedback
        setMotorsEnabled(true);  // enable motors
        // query initial motor status
        queryMotorStatus(0);
        queryMotorStatus(1);
        queryMotorStatus(2);
        queryMotorStatus(3);

        // subscriber to cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&ClearcoreDriverNode::cmdVelCallback, this, std::placeholders::_1));

        // publisher for feedback
        feedback_pub_ = this->create_publisher<std_msgs::msg::String>("clearcore/feedback", 10);

        // timer to poll serial for feedback
        timer_ = this->create_wall_timer(50ms, std::bind(&ClearcoreDriverNode::serialRead, this));

        RCLCPP_INFO(get_logger(), "ClearcoreDriverNode started");
    }
    
    ~ClearcoreDriverNode() {
        if (serial_.isOpen()) {
            serial_.close();
            RCLCPP_INFO(get_logger(), "Serial connection closed");
        }
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // simple diff‑drive mapping
        double v =  msg->linear.x;
        double w =  msg->angular.z;
        double wheel_circumference = 3.141592653589793 * wheel_diameter_;
        double steps_per_meter = steps_per_rev_ / wheel_circumference;
        int vel_left  = static_cast<int>((v - w * track_width_ / 2.0) * steps_per_meter);
        int vel_right = static_cast<int>((v + w * track_width_ / 2.0) * steps_per_meter);

        std::string cmd0 = "v0 " + std::to_string(vel_left)  + "\n";
        std::string cmd1 = "v1 " + std::to_string(vel_left) + "\n";
        std::string cmd2 = "v2 " + std::to_string(vel_right) + "\n";
        std::string cmd3 = "v3 " + std::to_string(vel_right) + "\n";

        serial_.write(cmd0);
        serial_.write(cmd1);
        serial_.write(cmd2);
        serial_.write(cmd3);
    }

    void serialRead() {
        if (!serial_.isOpen()) return;
        // read one line if available
        if (serial_.available()) {
            std::string line = serial_.readline(65536, "\n");
            auto msg = std_msgs::msg::String();
            msg.data = line;
            feedback_pub_->publish(msg);
        }
    }

    void setMotorsEnabled(bool enable) {
        std::string cmd0 = enable ? "e0\n" : "d0\n";
        std::string cmd1 = enable ? "e1\n" : "d1\n";
        std::string cmd2 = enable ? "e2\n" : "d2\n";
        std::string cmd3 = enable ? "e3\n" : "d3\n";
        serial_.write(cmd0);
        serial_.write(cmd1);
        serial_.write(cmd2);
        serial_.write(cmd3);
    }

    void setVerboseFeedback(bool enable) {
        std::string cmd = enable ? "f 1\n" : "f 0\n";
        serial_.write(cmd);
    }

    void queryMotorStatus(int motor_id) {
        std::string cmd = "q" + std::to_string(motor_id) + "s\n";
        serial_.write(cmd);
    }

    // --- members ---
    serial::Serial serial_;
    std::string port_;
    int baudrate_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // robot-specific constants (tune these)
    const double track_width_      = 0.585795;    // meters between left/right wheels
    const double steps_per_rev_   = 800.0;
    const double wheel_diameter_  = 0.230;  // in meters
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClearcoreDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}