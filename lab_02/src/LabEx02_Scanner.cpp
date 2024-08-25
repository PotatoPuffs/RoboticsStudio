#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserScanProcessor : public rclcpp::Node
{
public:
    LaserScanProcessor()
    : Node("laser_scan_processor") // Initialize the node with the name "laser_scan_processor"
    {
        // Subscribe to the laser scan topic published by Turtlebot3
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanProcessor::laser_callback, this, std::placeholders::_1));

        // Create a publisher for the modified subset of the laser scan
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/subset_scan", 10);
    }

private:
    // Callback function to handle incoming laser scan data
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Display the range reading for a specific angle (example: 180 degrees)
        int angle_index = 180; // Specify the index corresponding to the angle (e.g., 180 degrees)
        if (angle_index < msg->ranges.size()) { // Ensure the index is within bounds
            // Print the range value at the specified angle index
            RCLCPP_INFO(this->get_logger(), "Range at %d degrees: %f meters", angle_index, msg->ranges[angle_index]);
        } else {
            // Warn if the specified index is out of bounds
            RCLCPP_WARN(this->get_logger(), "Angle index out of bounds");
        }

        // Create a new LaserScan message to hold the subset of ranges
        auto subset_scan = sensor_msgs::msg::LaserScan();
        subset_scan.header = msg->header; // Copy the header from the original scan
        subset_scan.angle_min = msg->angle_min; // Copy the minimum angle
        subset_scan.angle_max = msg->angle_max; // Copy the maximum angle
        subset_scan.angle_increment = msg->angle_increment; // Copy the angle increment
        subset_scan.time_increment = msg->time_increment; // Copy the time increment
        subset_scan.scan_time = msg->scan_time; // Copy the scan time
        subset_scan.range_min = msg->range_min; // Copy the minimum range
        subset_scan.range_max = msg->range_max; // Copy the maximum range

        // Define the range of indices for the subset (e.g., 100 to 200)
        int start_index = 100;
        int end_index = 200;

        // Ensure the end index is within bounds
        if (end_index < msg->ranges.size()) {
            // Extract the subset of range values from the original scan
            subset_scan.ranges = std::vector<float>(msg->ranges.begin() + start_index, msg->ranges.begin() + end_index);
            // Extract the corresponding subset of intensity values
            subset_scan.intensities = std::vector<float>(msg->intensities.begin() + start_index, msg->intensities.begin() + end_index);

            // Print out the angles and their corresponding range values within the subset
            for (int i = start_index; i < end_index; ++i) {
                // Calculate the angle for the current index
                float angle = msg->angle_min + i * msg->angle_increment;
                // Retrieve the corresponding range value
                float range = msg->ranges[i];
                // Print the angle (in degrees) and range value
                RCLCPP_INFO(this->get_logger(), "Angle: %f degrees, Range: %f meters", angle * 180.0 / M_PI, range);
            }

            // Publish the subset of the laser scan on the new topic "/subset_scan"
            publisher_->publish(subset_scan);
        } else {
            // Warn if the end index is out of bounds
            RCLCPP_WARN(this->get_logger(), "End index out of bounds");
        }
    }

    // Subscriber to receive laser scan messages
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    // Publisher to publish the modified subset of laser scan data
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize ROS2
    rclcpp::spin(std::make_shared<LaserScanProcessor>()); // Spin the node to process callbacks
    rclcpp::shutdown(); // Shutdown ROS2
    return 0;
}

