#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>
#include <limits>

// Global publishers
rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cylinder_position_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

// Function to calculate the distance between two points in the scan
double calculateDistance(double range1, double range2, double angle_diff) {
    return sqrt(range1 * range1 + range2 * range2 - 2 * range1 * range2 * cos(angle_diff));
}

// Function to filter out the cylindrical object from laser scan data
std::vector<float> filterCylindricalObject(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    std::vector<float> filtered_ranges = scan->ranges;
    double object_diameter = 0.3; // Diameter of the object (30 cm)

    // Loop through the laser scan data to find the cylindrical object
    for (size_t i = 1; i < scan->ranges.size(); ++i) {
        double angle_diff = scan->angle_increment;
        double distance = calculateDistance(scan->ranges[i - 1], scan->ranges[i], angle_diff);

        // Check if the detected object matches the known diameter
        if (distance < object_diameter + 0.05 && distance > object_diameter - 0.05) {
            // Assume this is part of the cylindrical object, filter it out
            filtered_ranges[i] = std::numeric_limits<float>::infinity();  // Filter the range
        }
    }

    return filtered_ranges;
}

// Callback function for laser scan messages
void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    sensor_msgs::msg::LaserScan filtered_scan = *scan;  // Copy the original scan
    filtered_scan.ranges = filterCylindricalObject(scan);  // Apply filtering

    // Publish the filtered scan
    filtered_scan_pub->publish(filtered_scan);

    // Optionally, publish the position of the cylindrical object in the map
    geometry_msgs::msg::PointStamped cylinder_position;
    // Assuming you know the position, populate cylinder_position here
    cylinder_position.header.frame_id = "map";
    cylinder_position.point.x = 2.0;  // Example position
    cylinder_position.point.y = 3.0;  // Example position
    cylinder_position.point.z = 0.0;  // Assuming object on ground

    cylinder_position_pub->publish(cylinder_position);

    // Visualization marker for RViz
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "cylinder";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cylinder_position.point.x;
    marker.pose.position.y = cylinder_position.point.y;
    marker.pose.position.z = 0.0;
    marker.scale.x = 0.3;  // Diameter
    marker.scale.y = 0.3;  // Diameter
    marker.scale.z = 1.0;  // Height
    marker.color.a = 1.0;  // Opacity
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker_pub->publish(marker);
}

int main(int argc, char** argv) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("cylinder_filter_node");

    // Log message indicating that the node has been initialized
    RCLCPP_INFO(node->get_logger(), "Node initialized: cylinder_filter_node");

    // Initialize publishers
    filtered_scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);
    cylinder_position_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("/cylinder_position", 10);
    marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // Initialize subscriber
    auto scan_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, laserCallback);

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
