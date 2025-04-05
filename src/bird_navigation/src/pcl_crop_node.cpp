#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

// Bring specific PCL classes and functions into the global namespace
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::fromROSMsg;
using pcl::toROSMsg;
using pcl::CropBox;

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

class SimpleNode : public Node {
public:
    SimpleNode() : Node("simple_node") {
        // Declare a parameter with a default value
        declare_parameter<string>("example_param", "default_value");
        example_param_ = get_parameter("example_param").as_string();
        RCLCPP_INFO(get_logger(), "Parameter 'example_param': %s", example_param_.c_str());

        // Create a subscription for input point clouds
        point_cloud_subscription_ = create_subscription<PointCloud2>(
            "input_cloud", 10, [this](const PointCloud2::SharedPtr msg) { processPointCloud(msg); });

        // Create a publisher for cropped point clouds
        cropped_publisher_ = create_publisher<PointCloud2>("cropped_cloud", 10);

        RCLCPP_INFO(get_logger(), "SimpleNode with PCL cropping has been started.");
    }

private:
    void processPointCloud(const PointCloud2::SharedPtr msg) {
        // Convert ROS PointCloud2 message to PCL PointCloud
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
        fromROSMsg(*msg, *cloud);

        // Apply a crop box filter
        CropBox<PointXYZ> crop_box_filter;
        crop_box_filter.setMin(Eigen::Vector4f(-0.75 / 2, -0.94, -1.0, 1.0)); // Define the min bounds
        crop_box_filter.setMax(Eigen::Vector4f(0.75 / 2, 0.5, 1.0, 1.0));   // Define the max bounds
        crop_box_filter.setInputCloud(cloud);

        PointCloud<PointXYZ>::Ptr cropped_cloud(new PointCloud<PointXYZ>());
        crop_box_filter.filter(*cropped_cloud);

        // Subtraction of cloud
        pcl::PointCloud<pcl::PointXYZ> result_cloud;
        for (const auto& point1 : cloud->points) {  // Use cloud->points to access the points
            bool found = false;
            // Check if the point is in cropped_cloud
            for (const auto& point2 : cropped_cloud->points) {  // Use cropped_cloud->points to access points
                if (point1.x == point2.x && point1.y == point2.y && point1.z == point2.z) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                result_cloud.push_back(point1); // Add point to result if not found in cropped_cloud
            }
        }

        // Convert PCL PointCloud back to ROS PointCloud2 message
        sensor_msgs::msg::PointCloud2 output_msg;  // Missing semicolon added

        toROSMsg(result_cloud, output_msg);
        output_msg.header = msg->header;  // Preserve the original header

        // Publish the cropped point cloud
        cropped_publisher_->publish(output_msg);
        RCLCPP_INFO(get_logger(), "Published cropped point cloud.");
    }

    string example_param_;
    Publisher<PointCloud2>::SharedPtr cropped_publisher_;
    Subscription<PointCloud2>::SharedPtr point_cloud_subscription_;
};

int main(int argc, char **argv) {
    init(argc, argv);
    auto node = make_shared<SimpleNode>();
    spin(node);
    shutdown();
    return 0;
}
