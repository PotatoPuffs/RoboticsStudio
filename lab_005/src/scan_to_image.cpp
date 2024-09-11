// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <opencv2/opencv.hpp>
// #include <vector>

// class ScanToImageNode : public rclcpp::Node {
// public:
//     ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientaion_(0.0) {
//         scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
//         cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//         RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
//     }

// private:
//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//         // Purpose: This function is a callback that gets triggered whenever a new LaserScan message is received from the /scan topic.

//         // Functionality:

//         //     Convert LaserScan to Image: The laserScanToMat function is called to convert the LaserScan data into an image (a cv::Mat).
//         //     Handle First Image: If it's the first time this function is being called, it captures and displays the first image.
//         //     Handle Second Image: If it's the second time this function is called, it captures and displays the second image, then calculates the change in orientation (yaw).
//         //     Update and Rotate: For subsequent calls, it updates the images, calculates the yaw change, and logs the relative orientation.
       
//         // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
//         cv::Mat scan_image = laserScanToMat(msg);
        
//         try
//         {
//             cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
//             RCLCPP_INFO(this->get_logger(), "Image received");
//             cv::Point center(image.cols / 2, image.rows / 2);
//             cv::circle(image, center, 50, cv::Scalar(0, 255, 0), 3);
//             auto modified_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();

//             image_pub_->publish(*modified_msg);
//         }
//         catch (cv_bridge::Exception& e)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//         }
//     }
 


//     cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        
//         // Purpose: Converts a LaserScan message into a binary image (cv::Mat), where each pixel represents the presence of an obstacle detected by the laser scanner.

//         // Functionality:

//         //      Create Image: Initializes a blank image of size 500x500 pixels.
//         //      Map Polar to Cartesian: Iterates over the laser scan data, converting polar coordinates (distance and angle) to Cartesian coordinates (x, y) and sets the corresponding pixel in the image to white (255) if within range.
    
//     }

//     void calculateYawChange() {
//         // Purpose: Estimates the change in orientation (yaw angle) of the robot by comparing two images.

//         // Functionality:

//         //     Feature Matching: Uses feature detection and matching to find corresponding points between the two images.
//                     // std::vector<cv::Point2f> srcPoints, dstPoints;
//                     // detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);
//         //     Estimate Transformation: Computes an affine transformation matrix to determine the rotation between the two images.
//         //     Calculate Angle: Extracts the rotation angle from the transformation matrix and converts it to degrees.
//     }

//     void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
//                                 std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
//         cv::Ptr<cv::ORB> orb = cv::ORB::create();
//         std::vector<cv::KeyPoint> keypoints1, keypoints2;
//         cv::Mat descriptors1, descriptors2;

//         orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
//         orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

//         cv::BFMatcher matcher(cv::NORM_HAMMING);
//         std::vector<cv::DMatch> matches;
//         matcher.match(descriptors1, descriptors2, matches);

//         // Sort matches based on distance (lower distance means better match)
//         std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
//             return a.distance < b.distance;
//         });

//         // Determine the number of top matches to keep (30% of total matches)
//         size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

//         // Keep only the best matches (top 30%)
//         std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

//         for (const auto& match : matches) {
//             srcPoints.push_back(keypoints1[match.queryIdx].pt);
//             dstPoints.push_back(keypoints2[match.trainIdx].pt);
//         }
//     }

//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

//     cv::Mat first_image_, second_image_;
//     bool first_image_captured_ = false;
//     bool second_image_captured_ = false;

//     double angle_difference_;
//     double relative_orientaion_ = 0.0;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ScanToImageNode>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ScanToImageNode : public rclcpp::Node {
public:
    ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientation_(0.0) {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Purpose: This function is a callback that gets triggered whenever a new LaserScan message is received from the /scan topic.

        // Convert the LaserScan message to an OpenCV image (binary image representing obstacles)
        cv::Mat image = laserScanToMat(msg);

        if (!first_image_captured_) {
            // Capture the first image
            first_image_ = image.clone();
            cv::imshow("First Image", first_image_);
            cv::waitKey(1);
            first_image_captured_ = true;
        } else if (!second_image_captured_) {
            // Capture the second image
            second_image_ = image.clone();
            cv::imshow("Second Image", second_image_);
            cv::waitKey(1);
            second_image_captured_ = true;
            
            // Now that we have both images, calculate the yaw change
            calculateYawChange();
        } else {
            // Update the images continuously, calculate and log the yaw difference
            first_image_ = second_image_;
            second_image_ = image.clone();
            calculateYawChange();
        }
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        // Create a blank image (500x500) to represent the laser scan
        int image_size = 500;
        cv::Mat img(image_size, image_size, CV_8UC1, cv::Scalar(0));

        // Center of the image
        int center_x = image_size / 2;
        int center_y = image_size / 2;

        // Iterate through the scan data and map laser ranges to pixels in the image
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            if (range < scan->range_min || range > scan->range_max) {
                continue;  // Ignore out-of-range values
            }

            // Convert the angle (from scan) to Cartesian coordinates (x, y)
            float angle = scan->angle_min + i * scan->angle_increment;
            int x = static_cast<int>(center_x + (range * cos(angle) * 100.0));
            int y = static_cast<int>(center_y + (range * sin(angle) * 100.0));

            // Set the corresponding pixel in the image to white (255) to mark an obstacle
            if (x >= 0 && x < image_size && y >= 0 && y < image_size) {
                img.at<uchar>(y, x) = 255;
            }
        }

        return img;
    }

    void calculateYawChange() {
        // Detect features and match them between the two images (first_image_ and second_image_)
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 4 || dstPoints.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Not enough feature points detected for reliable matching.");
            return;
        }

        // Estimate an affine transformation matrix between the two images
        cv::Mat transform = cv::estimateAffinePartial2D(srcPoints, dstPoints);

        if (!transform.empty()) {
            // Extract the rotation component from the transformation matrix
            double angle_radians = atan2(transform.at<double>(1, 0), transform.at<double>(0, 0));
            double angle_degrees = angle_radians * 180.0 / CV_PI;

            // Calculate the relative orientation change
            relative_orientation_ += angle_degrees;
            RCLCPP_INFO(this->get_logger(), "Yaw change (degrees): %f, Total Orientation: %f", angle_degrees, relative_orientation_);
        }
    }

    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        // ORB feature detector and descriptor extractor
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        // Detect and compute ORB features in both images
        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        // Use a brute-force matcher with Hamming distance for ORB descriptors
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance = better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Keep only the best 15% of matches
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        // Extract the matched keypoints
        for (const auto& match : goodMatches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat first_image_, second_image_;
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;

    double angle_difference_;
    double relative_orientation_ = 0.0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToImageNode>());
    rclcpp::shutdown();
    return 0;
}
