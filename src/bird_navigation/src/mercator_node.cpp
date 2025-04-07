#include <memory>
#include <mutex>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <Eigen/Dense>

using std::placeholders::_1;

class Mercator : public rclcpp::Node {
public:
Mercator() : Node("mercator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
  input_topic_ = this->declare_parameter("input_topic", "/local_grid");
  output_topic_ = this->declare_parameter("output_topic", "/map");
  base_frame_ = this->declare_parameter("base_frame", "base_link");
  odom_frame_ = this->declare_parameter("odom_frame", "odom");
  map_frame_ = this->declare_parameter("map_frame", "map");

  // Initialize the transform to identity so it can be published immediately.
  map_to_odom_.header.frame_id = map_frame_;
  map_to_odom_.child_frame_id = odom_frame_;
  map_to_odom_.transform.translation.x = 0.0;
  map_to_odom_.transform.translation.y = 0.0;
  map_to_odom_.transform.translation.z = 0.0;
  map_to_odom_.transform.rotation.x = 0.0;
  map_to_odom_.transform.rotation.y = 0.0;
  map_to_odom_.transform.rotation.z = 0.0;
  map_to_odom_.transform.rotation.w = 1.0;

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, 10);
  grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    input_topic_, 10, std::bind(&Mercator::grid_callback, this, _1));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Mercator::publish_map_to_odom, this));
  RCLCPP_INFO(this->get_logger(),  "Mercator initialized with initial transform published.");
}

private:
  // Converts a log odds value to an occupancy probability between 0 and 100
  int logOddsToOccupancy(double log_odds) {
    double odds = std::exp(log_odds);
    double p = odds / (1 + odds);
    int occ = static_cast<int>(std::round(p * 100));
    // Clamp to valid values: if p is too uncertain, use -1.
    return (p < 0.05 || p > 0.95) ? occ : occ;
  }

  void grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Instead of looking up the transform, use the published transform.
    geometry_msgs::msg::TransformStamped odom_in_map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      odom_in_map = map_to_odom_;
    }
    // Optionally, if the transform is still identity (or not updated) you can add a check here.
    // e.g., if (odom_in_map.transform.rotation.w == 0.0) { ... }

    // Convert to tf2::Transform if inversion is needed.
    tf2::Transform tf_odom_in_map;
    tf2::fromMsg(odom_in_map.transform, tf_odom_in_map);
    // Invert to get map -> odom (if your integration logic expects that)
    tf2::Transform tf_map_to_odom = tf_odom_in_map.inverse();
    tf2::Quaternion q_map_to_odom = tf_map_to_odom.getRotation();
    tf2::Vector3 trans_map_to_odom = tf_map_to_odom.getOrigin();

    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      // Update map_to_odom_ with the inverted transform (optional, if you want to update it)
      map_to_odom_.header.stamp = this->get_clock()->now();
      map_to_odom_.transform.translation.x = trans_map_to_odom.x();
      map_to_odom_.transform.translation.y = trans_map_to_odom.y();
      map_to_odom_.transform.translation.z = trans_map_to_odom.z();
      map_to_odom_.transform.rotation = tf2::toMsg(q_map_to_odom);

      if (!master_map_) {
          // Initialize master map from the first message.
          master_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);
          master_map_->header.frame_id = map_frame_;
          // Initialize the log-odds vector with 0 (i.e. p=0.5)
          master_log_odds_.resize(master_map_->data.size(), 0.0);
      } else {
          // Use the stored transform for integration.
          integrate_grid(*msg, odom_in_map);
      }

      master_map_->header.stamp = this->get_clock()->now();
      map_pub_->publish(*master_map_);
    }
  }

  void integrate_grid(const nav_msgs::msg::OccupancyGrid &local_grid, 
                      const geometry_msgs::msg::TransformStamped &transform) {
    // Log odds update parameters
    const double log_odds_occ = std::log(0.7 / 0.3);
    const double log_odds_free = std::log(0.3 / 0.7);
    int local_width = local_grid.info.width;
    int local_height = local_grid.info.height;
    double resolution = master_map_->info.resolution;
    int global_width = master_map_->info.width;
    int global_height = master_map_->info.height;

    // Determine the origin in master map coordinates using the provided transform
    int origin_x = static_cast<int>((transform.transform.translation.x - master_map_->info.origin.position.x) / resolution);
    int origin_y = static_cast<int>((transform.transform.translation.y - master_map_->info.origin.position.y) / resolution);

    for (int y = 0; y < local_height; ++y) {
      for (int x = 0; x < local_width; ++x) {
        int mx = origin_x + x;
        int my = origin_y + y;
        if (mx >= 0 && mx < global_width && my >= 0 && my < global_height) {
          int idx_local = y * local_width + x;
          int idx_global = my * global_width + mx;
          int8_t val = local_grid.data[idx_local]; // -1 unknown, 0-100 valid
          if (val != -1) {
            // Use 50 as the threshold for occupancy
            if (val >= 50) {
              master_log_odds_[idx_global] += log_odds_occ;
            } else {
              master_log_odds_[idx_global] += log_odds_free;
            }
            // Convert updated log odds to occupancy probability
            master_map_->data[idx_global] = logOddsToOccupancy(master_log_odds_[idx_global]);
          }
        }
      }
    }
  }

  void publish_map_to_odom() {
    map_to_odom_.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(map_to_odom_);
  }

  std::string input_topic_, output_topic_, base_frame_, odom_frame_, map_frame_;
  nav_msgs::msg::OccupancyGrid::SharedPtr master_map_;
  std::vector<double> master_log_odds_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped map_to_odom_;
  std::mutex map_mutex_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mercator>());
  rclcpp::shutdown();
  return 0;
}