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

class LinacDriverNode : public rclcpp::Node {
public:
    LinacDriverNode() : Node("linac_driver_node") {

         // parameters
         this->declare_parameter<std::string>("port", "/dev/ttyACM1");
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

        zpt_sub_ = this->create_subscription<std_msgs::msg::Bool>("driver_enable", 10, std::bind(&LinacDriverNode::bool_callback, this, std::placeholders::_1));
        bucket_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("bucket_cmd", 10, std::bind(&LinacDriverNode::bucket_callback, this, std::placeholders::_1));


    }

private:
    void bool_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received driver_enable: %s", msg->data ? "true" : "false");
        // Add functionality based on the received message
    }

    void bucket_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        static int prev_state = -1;
        int current_state = (msg->linear.z > 0.0) ? 0 :
                            (msg->linear.z < 0.0) ? 1 :
                            (msg->angular.y > 0.0) ? 2 :
                            (msg->angular.y < 0.0) ? 3 : 4;

        if (current_state != prev_state) {
            const char* state_msgs[] = {
                "Linear Z Positive",
                "Linear Z Negative",
                "Angular Y Positive",
                "Angular Y Negative"
            };
            if (current_state < 4) {
                RCLCPP_INFO(get_logger(), "State changed: %s", state_msgs[current_state]);
            }
            prev_state = current_state;
        }
    }

    serial::Serial serial_;
    std::string port_;
    int baudrate_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zpt_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr bucket_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LinacDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}