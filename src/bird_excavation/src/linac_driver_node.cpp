#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "bird_interfaces/srv/bucket_pos.hpp"
#include <serial/serial.h>
#include <chrono>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

class LinacDriverNode : public rclcpp::Node {
public:
    LinacDriverNode() : Node("linac_driver_node") {

         // parametersd
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

        cal_srv_ = this->create_service<std_srvs::srv::Trigger>("linac_driver_node/calibrate",std::bind(&LinacDriverNode::cal_service_callback, this, std::placeholders::_1, std::placeholders::_2));
        buck_pos_srv_ = this->create_service<bird_interfaces::srv::BucketPos>("linac_driver_node/bucket_pos",std::bind(&LinacDriverNode::bucket_pos_callback, this, std::placeholders::_1, std::placeholders::_2));
        bucket_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&LinacDriverNode::bucket_callback, this, std::placeholders::_1));


    }

    ~LinacDriverNode() {
        // Close serial connection

        try {
            serial_.write("STATE:IDLE\n");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Failed to send shutdown command: %s", e.what());
        }

        if (serial_.isOpen()) {
            serial_.close();
            RCLCPP_INFO(get_logger(), "Serial connection closed");
        }
    }

private:
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
            const char* pos_msgs[] = {
            "POS:0,450",
            "POS:0,0",
            "POS:3600,2500",
            "POS:200,200"
            };
            if (current_state < 4) {
            RCLCPP_INFO(get_logger(), "State changed: %s", state_msgs[current_state]);
            try {
                serial_.write(std::string(pos_msgs[current_state]) + "\n");
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "Failed to send serial message: %s", e.what());
            }
            }
            prev_state = current_state;
        }
    }

    void cal_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        (void)request; // unused
        try {
            serial_.write("STATE:IDLE\n");
            serial_.write("STATE:ACTIVE\n");
            serial_.write("CAL:ARM\n");
            RCLCPP_INFO(get_logger(), "Calibration commands sent over serial.");
            response->success = true;
            response->message = "Calibration commands sent.";
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Failed to send calibration commands: %s", e.what());
            response->success = false;
            response->message = e.what();
        }
    }

    void bucket_pos_callback(
    const std::shared_ptr<bird_interfaces::srv::BucketPos::Request> request,
    std::shared_ptr<bird_interfaces::srv::BucketPos::Response> response)
{
    static const char* pos_cmds[] = {
        "POS:0,450",      // Excavation
        "POS:3600,2600",  // Deposit
        "POS:400,0",       // Low Drive
        "POS:1200,0"     // High Drive
        
    };

    int idx = request->position;
    if (idx < 0 || idx > 3) {
        response->success = false;
        response->message = "Invalid position index";
        RCLCPP_WARN(get_logger(), "Invalid bucket position index: %d", idx);
        return;
    }

    try {
        serial_.write(std::string(pos_cmds[idx]) + "\n");
        response->success = true;
        response->message = std::string("Sent: ") + pos_cmds[idx];
        RCLCPP_INFO(get_logger(), "Sent bucket position: %s", pos_cmds[idx]);
    } catch (const std::exception &e) {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(get_logger(), "Failed to send bucket position: %s", e.what());
    }
}

    serial::Serial serial_;
    std::string port_;
    int baudrate_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cal_srv_;
    rclcpp::Service<bird_interfaces::srv::BucketPos>::SharedPtr buck_pos_srv_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr bucket_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LinacDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}