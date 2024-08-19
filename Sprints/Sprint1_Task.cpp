#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserScanning : public rclcpp::Node
{
public:
    LaserScanning() : Node("laser_scan")
    {
        // Subscribe to the /scan topic
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanning::scanCallback, this, std::placeholders::_1));

        // Publisher for the processed scan data
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_processed", 10);

        // Set the value for n (e.g., every nth point)
        n_ = 5;  // This value can be modified based on your requirement
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Create a new LaserScan message for publishing
        auto new_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

        // Clear the ranges array and populate with every nth point
        std::vector<float> new_ranges;
        for (size_t i = 0; i < msg->ranges.size(); i += n_)
        {
            new_ranges.push_back(msg->ranges[i]);
        }

        // Update the range in the new scan
        new_scan->ranges = new_ranges;

        // Print the modified laser scan data
        RCLCPP_INFO(this->get_logger(), "Processed scan data:");
        for (size_t i = 0; i < new_ranges.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "[%zu]: %f", i * n_, new_ranges[i]);
        }

        // Publish the modified scan
        scan_pub_->publish(*new_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    int n_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanning>());
    rclcpp::shutdown();
    return 0;
}
