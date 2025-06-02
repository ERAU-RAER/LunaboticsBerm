#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <serial/serial.h>
#include <chrono>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

class ClearcoreDriverNode : public rclcpp::Node
{
public:
    ClearcoreDriverNode() : Node("clearcore_driver_node") {
        // parameters
        this->declare_parameter<std::string>("port", "/dev/serial/by-id/usb-Teknic__Inc._Teknic_ClearCore_31B0D2CB534D394852202020FF044536-if00");
        this->declare_parameter<int>("baudrate", 2000000);
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

        // subscriber to cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&ClearcoreDriverNode::cmdVelCallback, this, std::placeholders::_1));
        // subscriber to zero point
        zeropt_sub_ = this->create_subscription<std_msgs::msg::Bool>("zeropt", 10, std::bind(&ClearcoreDriverNode::zeroptCallback, this, std::placeholders::_1));

        cal_sub_ = this->create_subscription<std_msgs::msg::Bool>("arm_cal", 10, std::bind(&ClearcoreDriverNode::zeroptCallback, this, std::placeholders::_1));

        // publisher for feedback
        feedback_pub_ = this->create_publisher<std_msgs::msg::String>("clearcore/feedback", 10);

        chatter_pub_ = this->create_publisher<std_msgs::msg::String>("clearcore/chatter", 10);

        disable_ser_ = this->create_service<std_srvs::srv::Trigger>(
            "clearcore_driver_node/disable_motors",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                (void)req;
                this->disableMotors();
                res->success = true;
                res->message = "Motors disabled";
            });

        enable_ser_ = this->create_service<std_srvs::srv::Trigger>(
            "clearcore_driver_node/enable_motors",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                (void)req;
                this->enableMotors();
                res->success = true;
                res->message = "Motors enabled";
            });

        // timer to poll serial for feedback
        timer_ = this->create_wall_timer(50ms, std::bind(&ClearcoreDriverNode::serialRead, this));

        RCLCPP_INFO(get_logger(), "ClearcoreDriverNode started");
    }
    
    ~ClearcoreDriverNode() {
        // Close serial connection
        if (serial_.isOpen()) {
            serial_.close();
            RCLCPP_INFO(get_logger(), "Serial connection closed");
        }
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Package linear and angular velocities into a command string (scaled by 1000)
        float v = msg->linear.x;
        float w = msg->angular.z;
        int i_v = std::lround(v * 1000);
        int i_w = std::lround(w * 1000);

        char cmd[64];
        int len = std::snprintf(cmd, sizeof(cmd), "v %d %d\r\n", i_v, i_w);
        if (len > 0) {
            RCLCPP_DEBUG(get_logger(), "Sending cmd: %s", cmd);
            serial_.write(std::string(cmd, len));
            auto chatter_msg = std_msgs::msg::String();
            chatter_msg.data = cmd;
            chatter_pub_->publish(chatter_msg);
            RCLCPP_DEBUG(get_logger(), "Published cmd to chatter: %s", cmd);
        }
    }

    void zeroptCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::string cmd = msg->data ? "z t\r\n" : "z 0\r\n";
        RCLCPP_DEBUG(get_logger(), "Sending zeropt cmd: %s", cmd.c_str());
        serial_.write(cmd);
        auto chatter_msg = std_msgs::msg::String();
        chatter_msg.data = cmd;
        chatter_pub_->publish(chatter_msg);
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

    // --- members ---
    serial::Serial serial_;
    std::string port_;
    int baudrate_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zeropt_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cal_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_ser_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_ser_;
    rclcpp::TimerBase::SharedPtr timer_;

    // robot-specific constants (tune these)
    const double track_width_     = 0.585795;    // meters between left/right wheels
    const double wheel_diameter_  = 0.230;  // in meters
    const double gear_ratio_      = 10.0;

    void disableMotors(u_int16_t motor = 255) {
        // If a specific motor [0-3] is provided, only disable that one;
        // Otherwise, disable all motors.
        if (motor < 4) {
            std::string cmd = "d" + std::to_string(motor) + "\r\n";
            RCLCPP_DEBUG(get_logger(), "Sending cmd: %s", cmd.c_str());
            serial_.write(cmd);
        } else {
            for (u_int16_t i = 0; i < 4; i++) {
                std::string cmd = "d" + std::to_string(i) + "\r\n";
                RCLCPP_DEBUG(get_logger(), "Sending cmd: %s", cmd.c_str());
                serial_.write(cmd);
            }
        }
    }

    void enableMotors(u_int16_t motor = 255) {
        // If a specific motor [0-3] is provided, only enable that one;
        // Otherwise, enable all motors.
        if (motor < 4) {
            std::string cmd = "e" + std::to_string(motor) + "\r\n";
            RCLCPP_DEBUG(get_logger(), "Sending cmd: %s", cmd.c_str());
            serial_.write(cmd);
        } else {
            for (u_int16_t i = 0; i < 4; i++) {
                std::string cmd = "e" + std::to_string(i) + "\r\n";
                RCLCPP_DEBUG(get_logger(), "Sending cmd: %s", cmd.c_str());
                serial_.write(cmd);
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClearcoreDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}