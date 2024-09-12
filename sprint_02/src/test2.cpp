//compares edge image and laser scan
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ScanToImageNode : public rclcpp::Node {
public:
    ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientation_(0.0) {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
    }

private:
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
        cv::Mat laser_image = laserScanToMat(msg);

        // Display the laser scan image
        cv::imshow("Laser Scan Image", laser_image);
        cv::waitKey(1);

        // Apply Gaussian blur and detect edges in the laser scan image
        cv::Mat blurred_image;
        cv::GaussianBlur(laser_image, blurred_image, cv::Size(5, 5), 0);

        cv::Mat edge_image;
        cv::Canny(blurred_image, edge_image, 50, 150);  // Apply Canny edge detection

        // Display the edge image
        cv::imshow("Edge Image", edge_image);
        cv::waitKey(1);

        // Optionally: Compare the laser scan image with the edge image
        compareLaserScanAndEdges(laser_image, edge_image);
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        // Convert LaserScan data to a binary cv::Mat image
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255;  // Set pixel to white (representing the laser scan point)
                }
            }
        }
        return image;
    }

    void compareLaserScanAndEdges(const cv::Mat& laser_image, const cv::Mat& edge_image) {
        // Detect and match features between the laser scan image and the edge image
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        // Use ORB feature detector and descriptor
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        orb->detectAndCompute(laser_image, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(edge_image, cv::noArray(), keypoints2, descriptors2);

        // Use BFMatcher to find feature matches
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance is better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Optionally keep the best matches
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        // Draw matches between the laser scan image and the edge image
        cv::Mat match_image;
        cv::drawMatches(laser_image, keypoints1, edge_image, keypoints2, goodMatches, match_image);

        // Display the matches
        cv::imshow("Matches between Laser Scan and Edge Image", match_image);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    double angle_difference_;
    double relative_orientation_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToImageNode>());
    rclcpp::shutdown();
    return 0;
}

