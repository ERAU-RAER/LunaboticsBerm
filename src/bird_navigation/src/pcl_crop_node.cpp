#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

// Bring specific PCL classes and functions into the global namespace
using pcl::PointCloud;
using pcl::PointXYZI;  // Use PointXYZI to carry intensity
using pcl::fromROSMsg;
using pcl::toROSMsg;
using pcl::CropBox;
using pcl::PassThrough;

// Bring specific rclcpp classes and functions into the global namespace
using rclcpp::Node;
using rclcpp::Publisher;
using rclcpp::Subscription;
using rclcpp::init;
using rclcpp::shutdown;
using rclcpp::spin;

// Bring specific std types into the global namespace
using std::string;
using std::make_shared;

// Bring specific sensor_msgs types into the global namespace
using sensor_msgs::msg::PointCloud2;

class CropNode : public Node {
public:
    CropNode() : Node("crop_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
        // Create a subscription for input point clouds
        point_cloud_subscription_ = create_subscription<PointCloud2>(
            "input_cloud", 10,
            [this](const PointCloud2::SharedPtr msg) { processPointCloud(msg); });

        // Create a publisher for cropped point clouds
        cropped_publisher_ = create_publisher<PointCloud2>("cloud_in", 10);

        RCLCPP_INFO(get_logger(), "Getting ready to CROP!!");
    }

private:
    void processPointCloud(const PointCloud2::SharedPtr msg) {
        double delta_z = 0.0;
        try {
            // Lookup transform with 4 arguments as required by ROS 2 Humble
            auto transformStamped = tf_buffer_.lookupTransform(
                "base_footprint", "livox_frame",
                rclcpp::Time(0),                         // time = latest available
                tf2::durationFromSec(0.1));              // timeout as tf2::Duration
            
            delta_z = transformStamped.transform.translation.z;
            // RCLCPP_INFO(get_logger(), "Delta Z between frames: %f", delta_z);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Could not transform livox_frame to base_footprint: %s", ex.what());
        }
        
        // Convert ROS PointCloud2 message to PCL PointCloud using PointXYZI
        PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>());
        fromROSMsg(*msg, *cloud);
        
        // Cut up the input cloud using adjusted z ranges with delta_z offset
        PointCloud<PointXYZI>::Ptr cloud_filtered_upper(new PointCloud<PointXYZI>());
        PointCloud<PointXYZI>::Ptr cloud_filtered_lower(new PointCloud<PointXYZI>());

        // Upper Band: subtract offset from original limits [0.05, 0.25]
        PassThrough<pcl::PointXYZI> pass_upper; 
        pass_upper.setInputCloud(cloud);
        pass_upper.setFilterFieldName("z");
        pass_upper.setFilterLimits(0.05 + delta_z, 0.25 + delta_z);
        pass_upper.filter(*cloud_filtered_upper);

        // Lower Band: subtract offset from original limits [-0.25, -0.05]
        PassThrough<pcl::PointXYZI> pass_lower; 
        pass_lower.setInputCloud(cloud);
        pass_lower.setFilterFieldName("z");
        pass_lower.setFilterLimits(-0.25 + delta_z, -0.05 + delta_z);
        pass_lower.filter(*cloud_filtered_lower);

        // Combine the filtered clouds
        PointCloud<PointXYZI>::Ptr cloud_filtered(new PointCloud<PointXYZI>());
        *cloud_filtered = *cloud_filtered_upper + *cloud_filtered_lower;

        // Use CropBox filter with negative flag to remove points inside the defined box
        CropBox<PointXYZI> crop_box_filter;
        crop_box_filter.setMin(Eigen::Vector4f(-0.75 / 2, -0.94, -1.0, 1.0)); // Define the min bounds
        crop_box_filter.setMax(Eigen::Vector4f(0.75 / 2, 0.5, 1.0, 1.0));     // Define the max bounds
        crop_box_filter.setInputCloud(cloud_filtered);
        crop_box_filter.setNegative(true); // Remove points inside the crop box
        crop_box_filter.filter(*cloud_filtered);
    
        // Convert PCL PointCloud back to ROS PointCloud2 message
        sensor_msgs::msg::PointCloud2 output_msg;
        toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header;  // Preserve the original header
    
        // Publish the cropped point cloud
        cropped_publisher_->publish(output_msg);
    }

    Publisher<PointCloud2>::SharedPtr cropped_publisher_;
    Subscription<PointCloud2>::SharedPtr point_cloud_subscription_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
    init(argc, argv);
    auto node = std::make_shared<CropNode>();
    spin(node);
    shutdown();
    return 0;
}