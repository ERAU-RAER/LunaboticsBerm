#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "bird_interfaces/srv/bucket_pos.hpp"

using std::placeholders::_1;

class JoyToCmdVel : public rclcpp::Node
{
public:
  JoyToCmdVel()
  : Node("joy_to_cmdvel"), toggle_state_(false), prev_button2_(0)
  {
    max_speed_ = this->declare_parameter("max_speed", 0.25);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyToCmdVel::joy_callback, this, _1));

    cmdvel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    zeropt_pub_ = this->create_publisher<std_msgs::msg::Bool>("zeropt", 10);
    cal_pub_ = this->create_publisher<std_msgs::msg::Bool>("arm_cal", 10);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // --- MOD LAYER ---
    if (msg->buttons[7] == 1) {
      static bool prev_mod_layer = false;
      if (!prev_mod_layer) {
        RCLCPP_INFO(this->get_logger(), "MOD LAYER: ACTIVE");
      }
      prev_mod_layer = true;

      // Calibrate service, only on rising edge of button 1
      if (msg->buttons[1] == 1 && prev_mod_button1_ == 0) {
        call_trigger_service("/linac_driver_node/calibrate");
      }
      prev_mod_button1_ = msg->buttons[1];

      if (prev_mod_axis1_ == 0){
        switch (int s = msg->axes[4])
        {
        case -1:
          call_bucket_pos_service(0);
          break;

        case 1:
          call_bucket_pos_service(1);
          break;
      
        default:
          break;
          (void)s;
        }
      }
      prev_mod_axis1_ = msg->axes[4];

      if (prev_mod_axis2_ == 0){
        switch (int s = msg->axes[5])
        {
        case -1:
          call_bucket_pos_service(2);
          break;

        case 1:
          call_bucket_pos_service(3);
          break;
      
        default:
          break;
          (void)s;
        }
      }
      prev_mod_axis2_ = msg->axes[5];
    }



    // --- DEFAULT LAYER ---
    else {
      geometry_msgs::msg::Twist twist_msg;

      constexpr double deadband = 0.0125;
      auto apply_deadband = [deadband](double val) {
        return std::abs(val) >= deadband ? val : 0.0;
      };

      twist_msg.linear.x = apply_deadband(msg->axes[1]) * max_speed_;
      twist_msg.angular.z = apply_deadband(msg->axes[0]) * max_speed_;
      twist_msg.linear.z = msg->axes[3];
      twist_msg.angular.y = msg->axes[2];

      // Button 2 toggle (index 2)
      if (msg->buttons[2] == 1 && prev_button2_ == 0) {
        toggle_state_ = !toggle_state_;
        std_msgs::msg::Bool toggle_msg;
        toggle_msg.data = toggle_state_;
        zeropt_pub_->publish(toggle_msg);
      }
      prev_button2_ = msg->buttons[2];

      // DEFAULT layer: disable/enable motors, only on rising edge of button 1
      if (msg->buttons[1] == 1 && prev_default_button1_ == 0) {
        call_trigger_service("/clearcore_driver_node/disable_motors");
        call_trigger_service("/clearcore_driver_node/enable_motors");
      }
      prev_default_button1_ = msg->buttons[1];

      // Only publish if twist values have changed
      if (!last_twist_ ||
          twist_msg.linear.x != last_twist_->linear.x ||
          twist_msg.angular.z != last_twist_->angular.z ||
          twist_msg.linear.z != last_twist_->linear.z ||
          twist_msg.angular.y != last_twist_->angular.y)
      {
        cmdvel_pub_->publish(twist_msg);
        last_twist_ = std::make_shared<geometry_msgs::msg::Twist>(twist_msg);
      }
    }
  }

  void call_trigger_service(const std::string &service_name) {
    auto client = this->create_client<std_srvs::srv::Trigger>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service %s not available.", service_name.c_str());
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request,
      [this, service_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
        if (result.get()->success) {
          RCLCPP_INFO(this->get_logger(), "Service %s called successfully.", service_name.c_str());
        } 
        else {
          RCLCPP_ERROR(this->get_logger(), "Failed to call service %s.", service_name.c_str());
        }
      }
    );
  }

  void call_bucket_pos_service(int pos) {
    // Replace with your actual service type and namespace if different
    using BucketPos = bird_interfaces::srv::BucketPos;
    auto client = this->create_client<BucketPos>("/linac_driver_node/bucket_pos");
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "BucketPos service not available.");
      return;
    }

    auto request = std::make_shared<BucketPos::Request>();
    request->position = pos;

    auto future = client->async_send_request(request,
      [this, pos](rclcpp::Client<BucketPos>::SharedFuture result) {
        if (result.get()->success) {
          RCLCPP_INFO(this->get_logger(), "BucketPos service called successfully with pos=%d.", pos);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to call BucketPos service with pos=%d.", pos);
        }
      }
    );
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zeropt_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cal_pub_;

  double max_speed_;
  bool toggle_state_;
  int prev_button2_;
  int prev_default_button1_ = 0;
  int prev_mod_button1_ = 0;
  int prev_mod_axis1_ = 0;
  int prev_mod_axis2_ = 0;

  std::shared_ptr<geometry_msgs::msg::Twist> last_twist_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCmdVel>());
  rclcpp::shutdown();
  return 0;
}
