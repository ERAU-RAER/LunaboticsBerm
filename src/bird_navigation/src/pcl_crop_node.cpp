#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Dense>

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
    CropNode() : Node("crop_node") {
        // Create a subscription for input point clouds
        point_cloud_subscription_ = create_subscription<PointCloud2>(
            "input_cloud", 10,
            [this](const PointCloud2::SharedPtr msg) { processPointCloud(msg); });

        // Create a publisher for cropped point clouds
        cropped_publisher_ = create_publisher<PointCloud2>("cropped_cloud", 10);

        RCLCPP_INFO(get_logger(), "Getting ready to CROP!!");
    }

private:
    void processPointCloud(const PointCloud2::SharedPtr msg) {
        // Convert ROS PointCloud2 message to PCL PointCloud using PointXYZI
        PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>());
        fromROSMsg(*msg, *cloud);
        
        // Cut up the input cloud
        PointCloud<PointXYZI>::Ptr cloud_filtered_upper(new PointCloud<PointXYZI>());
        PointCloud<PointXYZI>::Ptr cloud_filtered_lower(new PointCloud<PointXYZI>());

        PassThrough<pcl::PointXYZI> pass_upper; // Upper Band
        pass_upper.setInputCloud(cloud);
        pass_upper.setFilterFieldName("z");
        pass_upper.setFilterLimits(0.05, 0.25);
        pass_upper.filter(*cloud_filtered_upper);

        PassThrough<pcl::PointXYZI> pass_lower; // Lower Band
        pass_lower.setInputCloud(cloud);
        pass_lower.setFilterFieldName("z");
        pass_lower.setFilterLimits(-0.25, -0.05);
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
        // RCLCPP_INFO(get_logger(), "*CROPPING!*");
    }

    Publisher<PointCloud2>::SharedPtr cropped_publisher_;
    Subscription<PointCloud2>::SharedPtr point_cloud_subscription_;
};

int main(int argc, char **argv) {
    init(argc, argv);
    auto node = make_shared<CropNode>();
    spin(node);
    shutdown();
    return 0;
}