#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

class SimpleNode : public rclcpp::Node {
public:
    SimpleNode() : Node("simple_node") {
        // Declare a parameter with a default value
        this->declare_parameter<std::string>("example_param", "default_value");
        example_param_ = this->get_parameter("example_param").as_string();
        RCLCPP_INFO(this->get_logger(), "Parameter 'example_param': %s", example_param_.c_str());

        // Create a publisher for cropped point clouds
        cropped_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cropped_cloud", 10);

        // Create a subscription for input point clouds
        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_cloud", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                processPointCloud(msg);
            });

        RCLCPP_INFO(this->get_logger(), "SimpleNode with PCL cropping has been started.");
    }

private:
    void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Apply a crop box filter
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setMin(Eigen::Vector4f(-1.0, -1.0, -1.0, 1.0)); // Define the min bounds
        crop_box_filter.setMax(Eigen::Vector4f(1.0, 1.0, 1.0, 1.0));   // Define the max bounds
        crop_box_filter.setInputCloud(cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        crop_box_filter.filter(*cropped_cloud);

        // Convert PCL PointCloud back to ROS PointCloud2 message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cropped_cloud, output_msg);
        output_msg.header = msg->header; // Preserve the original header

        // Publish the cropped point cloud
        cropped_publisher_->publish(output_msg);
        RCLCPP_INFO(this->get_logger(), "Published cropped point cloud.");
    }

    std::string example_param_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}