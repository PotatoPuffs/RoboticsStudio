/**
 * @file cylinder_detector.cpp
 * @brief A ROS2 node to detect cylindrical objects from laser scan data, transform the position to the map frame, and visualize in RViz.
 * 
 * This node subscribes to laser scan data, identifies a cylindrical object of known diameter, transforms its position 
 * from the base_link frame to the map frame using tf2, and publishes the object's pose and a visualization marker.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>

/**
 * @class CylinderDetector
 * @brief A ROS2 node for detecting cylindrical objects using laser scans and visualizing them in RViz.
 * 
 * This class subscribes to the /scan topic for laser scan data, identifies cylindrical objects based on their radius, 
 * and publishes the object's pose and visualization marker after transforming the position from the base_link frame to the map frame.
 */
class CylinderDetector : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the CylinderDetector node.
   * 
   * Initializes the node, sets up subscriptions and publishers, and initializes the tf2 buffer and listener.
   */
  CylinderDetector() : Node("cylinder_detector"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // Subscribe to the laser scan topic
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&CylinderDetector::scan_callback, this, std::placeholders::_1));

    // Publisher for the identified cylindrical object's pose
    cylinder_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cylinder_pose", 10);
    
    // Publisher for visualization markers
    marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_; ///< Subscription to the laser scan topic.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cylinder_pose_publisher_; ///< Publisher for the cylinder's pose.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub; ///< Publisher for the RViz marker.

  tf2_ros::Buffer tf_buffer_; ///< Buffer for storing tf2 transformations.
  tf2_ros::TransformListener tf_listener_; ///< Listener for tf2 transformations.

  /**
   * @brief Callback function for processing incoming laser scan data.
   * 
   * This function iterates through the laser scan data, identifies potential cylindrical objects,
   * and publishes their positions after transforming to the map frame.
   * 
   * @param msg A shared pointer to the laser scan message.
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const float cylinder_diameter = 0.30;  // Known diameter of the cylindrical object in meters
    const float cylinder_radius = cylinder_diameter / 2.0;

    // Iterate through the laser scan ranges
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float range = msg->ranges[i];

      // Identify cylindrical-shaped object using simple logic (e.g., checking range consistency)
      if (is_cylinder(msg, i, cylinder_radius)) {
        // Calculate the position of the detected object in the robot's frame of reference (base_link)
        float angle = msg->angle_min + i * msg->angle_increment;
        float x = range * std::cos(angle);
        float y = range * std::sin(angle);

        // Publish the detected object's position with transformation to map frame
        publish_cylinder_position(x, y);
        break;  // Stop after detecting one object (can be modified if multiple objects need detection)
      }
    }
  }

  /**
   * @brief Function to check if the range data corresponds to a cylindrical object.
   * 
   * This function checks the consistency of the range data around a given index to determine if it matches the
   * known diameter of the cylindrical object.
   * 
   * @param msg A shared pointer to the laser scan message.
   * @param index The index in the laser scan range array to check.
   * @param cylinder_radius The known radius of the cylindrical object.
   * @return True if the object matches the cylindrical shape, false otherwise.
   */
  bool is_cylinder(const sensor_msgs::msg::LaserScan::SharedPtr& msg, size_t index, float cylinder_radius) {
    const int num_points = 10;  // Number of laser points around the object to consider
    int half_num_points = num_points / 2;
    float range_avg = 0.0;

    for (int i = -half_num_points; i <= half_num_points; ++i) {
      int idx = index + i;
      if (idx < 0 || idx >= static_cast<int>(msg->ranges.size())) {
        return false;
      }
      range_avg += msg->ranges[idx];
    }

    range_avg /= num_points;
    float estimated_diameter = 2 * range_avg * std::tan(num_points * msg->angle_increment / 2.0);

    return std::abs(estimated_diameter - cylinder_radius * 2) < 0.05;  // Tolerance for matching
  }

  /**
   * @brief Publishes the detected object's pose after transforming to the map frame.
   * 
   * This function takes the position of the detected object in the base_link frame, transforms it to the map frame,
   * and publishes the pose. If the transformation fails, a warning is logged.
   * 
   * @param x The x-coordinate of the detected object in the base_link frame.
   * @param y The y-coordinate of the detected object in the base_link frame.
   */
  void publish_cylinder_position(float x, float y) {
    // Create the pose of the identified cylinder in the base_link frame
    geometry_msgs::msg::PoseStamped cylinder_pose;
    cylinder_pose.header.frame_id = "base_link";  // Detected in robot's local frame
    cylinder_pose.pose.position.x = x;
    cylinder_pose.pose.position.y = y;
    cylinder_pose.pose.position.z = 0.0;
    cylinder_pose.pose.orientation.w = 1.0;  // No rotation

    try {
      // Transform the pose from base_link to map frame
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf_buffer_.transform(cylinder_pose, transformed_pose, "map");

      // Now, publish the transformed position in the map frame
      cylinder_pose_publisher_->publish(transformed_pose);

      RCLCPP_INFO(this->get_logger(), "Cylinder detected at: x = %.2f, y = %.2f in map frame", 
                  transformed_pose.pose.position.x, transformed_pose.pose.position.y);

      // Publish the visualization marker for RViz
      publish_marker(transformed_pose.pose.position.x, transformed_pose.pose.position.y);
      
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform cylinder position to map frame: %s", ex.what());
    }
  }

  /**
   * @brief Publishes a visualization marker for RViz at the given coordinates.
   * 
   * This function publishes a cylindrical marker at the given x, y coordinates in the map frame.
   * 
   * @param x The x-coordinate of the detected object in the map frame.
   * @param y The y-coordinate of the detected object in the map frame.
   */
  void publish_marker(float x, float y) {
    // Publish the visualization marker for RViz
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";  // Marker will be in the map frame
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "cylinder";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;  // On the ground plane
    marker.scale.x = 0.3;  // Diameter of the cylinder
    marker.scale.y = 0.3;  // Diameter of the cylinder
    marker.scale.z = 1.0;  // Height of the cylinder
    marker.color.a = 1.0;  // Opacity
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker_pub->publish(marker);
  }
};

/**
 * @brief Main function that initializes and runs the CylinderDetector node.
 * 
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit status.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CylinderDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
