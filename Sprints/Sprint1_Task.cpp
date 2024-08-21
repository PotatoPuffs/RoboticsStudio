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

        // Set the value for n (every nth point)
        n_ = 2;  // This value can be modified based on your requirement
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
       // Create a new LaserScan message for publishing
        auto new_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

        // Initialize the new ranges and intensities with infinity (or NaN)
        std::vector<float> new_ranges(msg->ranges.size(), std::numeric_limits<float>::infinity());
        std::vector<float> new_intensities(msg->intensities.size(), std::numeric_limits<float>::quiet_NaN());

        for (size_t i = 0; i < msg->ranges.size(); i += n_) {
            new_ranges[i] = msg->ranges[i];
            if (!msg->intensities.empty()) {
                new_intensities[i] = msg->intensities[i];
            }
        }

        // Update the ranges and intensities in the new scan
        new_scan->ranges = new_ranges;
        if (!msg->intensities.empty()) {
            new_scan->intensities = new_intensities;
        }

        // Print the modified laser scan data
        RCLCPP_INFO(this->get_logger(), "Processed scan data:");
        for (size_t i = 0; i < new_ranges.size(); ++i) {
            if (i % n_ == 0) {
                RCLCPP_INFO(this->get_logger(), "[Angle Index %zu]: Range = %f, Intensity = %f", 
                            i, new_ranges[i], new_intensities[i]);
            }
        }

        // Publish the modified scan
        scan_pub_->publish(*new_scan);

    }
    //Definitions
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    int n_;
};

// Initialisation
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanning>());
    rclcpp::shutdown();
    return 0;
}
