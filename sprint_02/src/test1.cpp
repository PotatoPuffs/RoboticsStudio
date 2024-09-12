//comparing laser scan
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
    // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
    cv::Mat img = laserScanToMat(msg);

    if (!first_image_captured_) {
        // Extract and display first image (map section)
        first_image_ = img.clone();
        first_image_captured_ = true;
        cv::imshow("Map Section Image", first_image_);
        cv::waitKey(1);

        // Apply Gaussian blur to smooth the image
        cv::Mat blurred_image;
        cv::GaussianBlur(first_image_, blurred_image, cv::Size(5, 5), 0);

        // Create edge image using Canny
        cv::Mat edge_image;
        cv::Canny(blurred_image, edge_image, 50, 150); // Adjust threshold values as needed

        // Display the edge image
        cv::imshow("Edge Image", edge_image);
        cv::waitKey(1);

        // Rotate the robot (optional)
        //rotateRobot();
    } else if (!second_image_captured_) {
        // Extract and display second image (laser scan)
        second_image_ = img.clone();
        second_image_captured_ = true;
        cv::imshow("Laser Scan Image", second_image_);
        cv::waitKey(1);

    } else {
        // Update images
        first_image_ = second_image_.clone();
        second_image_ = img.clone();
        cv::imshow("Laser Scan Image", second_image_);
        cv::waitKey(1);

        // Estimate yaw change and display matching features
        calculateYawChange();

        // Declare keypoints and matches
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        std::vector<cv::DMatch> goodMatches;

        // Detect and match features
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints, keypoints1, keypoints2, goodMatches);

        // Draw and display matched features
        cv::Mat match_image;
        cv::drawMatches(first_image_, keypoints1, second_image_, keypoints2, goodMatches, match_image);
        cv::imshow("Matches", match_image);
        cv::waitKey(1);

        relative_orientation_ += angle_difference_;
        RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f", relative_orientation_);
    }
}



    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        // Convert LaserScan data to cv::Mat (Image)
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
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        return image;
    }

    void calculateYawChange() {
    // Step 5: Detect and match features between Image B and Image C
    std::vector<cv::Point2f> srcPoints, dstPoints;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    std::vector<cv::DMatch> goodMatches;

    // Updated detectAndMatchFeatures call with keypoints1, keypoints2, and goodMatches
    detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints, keypoints1, keypoints2, goodMatches);

    if (srcPoints.size() < 3 || dstPoints.size() < 3) {
        RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
        return;
    }

    try {
        cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
        if (transform_matrix.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
        } else {
            // Extract the rotation angle from the transformation matrix
            angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
            angle_difference_ = angle_difference_ * 180.0 / CV_PI;
            RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
        }
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
    }
}


    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                            std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints,
                            std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2,
                            std::vector<cv::DMatch>& goodMatches) {
    // Detect and match features between two images
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    cv::Mat descriptors1, descriptors2;

    orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
    orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    // Sort matches based on distance (lower distance means better match)
    std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
        return a.distance < b.distance;
    });

    // Determine the number of top matches to keep (15% of total matches)
    size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

    // Keep only the best matches (top 15%)
    goodMatches.assign(matches.begin(), matches.begin() + numGoodMatches);

    for (const auto& match : goodMatches) {
        srcPoints.push_back(keypoints1[match.queryIdx].pt);
        dstPoints.push_back(keypoints2[match.trainIdx].pt);
    }
}


    // // Optional: Rotate robot
    // void rotateRobot() {
    //     auto twist_msg = geometry_msgs::msg::Twist();
    //     twist_msg.angular.z = 1.0;  // Rotate with some angular velocity
    //     cmd_publisher_->publish(twist_msg);

    //     // Sleep for a while to allow the robot to rotate
    //     rclcpp::sleep_for(std::chrono::seconds(2));

    //     // Stop rotation
    //     twist_msg.angular.z = 0.0;
    //     cmd_publisher_->publish(twist_msg);
    // }

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
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <opencv2/opencv.hpp>
// #include <vector>

// class ScanToImageNode : public rclcpp::Node {
// public:
//     ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientation_(0.0) {
//         scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
//         cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//         RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
//     }

// private:
//     // Laser scan callback to process the incoming scan data
//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//         // Convert LaserScan to cv::Mat (Image C)
//         cv::Mat laser_image = laserScanToMat(msg);

//         if (!first_image_captured_) {
//             // Step 2: Extract section of the map around the robot (Image A)
//             first_image_ = extractMapSectionAroundRobot();
//             first_image_captured_ = true;
//             cv::imshow("Map Section Image", first_image_);
//             cv::waitKey(1);

//             // Step 3: Apply edge detection to Image A to get Image B
//             cv::Mat edge_image;
//             cv::Canny(first_image_, edge_image, 50, 150);  // Image B: Edge-detected map section
//             cv::imshow("Edge Image (Image B)", edge_image);
//             cv::waitKey(1);

//             // Save edge_image for future comparisons
//             edge_image_ = edge_image;

//             // Optionally move or rotate the robot
//             //rotateRobot();
//         } else {
//             // Step 4: Display Laser scan image (Image C)
//             cv::imshow("Laser Scan Image (Image C)", laser_image);
//             cv::waitKey(1);

//             // Step 5: Compare Image B (edge_image_) and Image C (laser_image) to estimate rotation (yaw)
//             calculateYawChange(edge_image_, laser_image);

//             // Rotate or move the robot based on yaw estimation
//             moveRobot();
//         }
//     }

//     // Function to move the robot based on the results of scan matching
//     void moveRobot() {
//         auto twist_msg = geometry_msgs::msg::Twist();
//         twist_msg.angular.z = angle_difference_ * 0.1;  // Scale the angular velocity based on yaw difference
//         cmd_publisher_->publish(twist_msg);

//         // Sleep for a short duration to allow the robot to move
//         rclcpp::sleep_for(std::chrono::milliseconds(500));

//         // Stop rotation after movement
//         twist_msg.angular.z = 0.0;
//         cmd_publisher_->publish(twist_msg);
//     }

//     // Function to estimate yaw change based on Image B and Image C comparison
//     void calculateYawChange(const cv::Mat& edge_image, const cv::Mat& laser_image) {
//         std::vector<cv::Point2f> srcPoints, dstPoints;
//         detectAndMatchFeatures(edge_image, laser_image, srcPoints, dstPoints);

//         if (srcPoints.size() < 3 || dstPoints.size() < 3) {
//             RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
//             return;
//         }

//         try {
//             cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
//             if (transform_matrix.empty()) {
//                 RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
//             } else {
//                 // Extract the rotation angle from the transformation matrix
//                 angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
//                 angle_difference_ = angle_difference_ * 180.0 / CV_PI;  // Convert radians to degrees
//                 RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
//             }
//         } catch (const cv::Exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
//         }
//     }

//     // Function to detect and match features between Image B (map edge) and Image C (laser scan)
//     void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
//                                 std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
//         cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);  // Increase the number of features for ORB
//         std::vector<cv::KeyPoint> keypoints1, keypoints2;
//         cv::Mat descriptors1, descriptors2;

//         orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
//         orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

//         cv::BFMatcher matcher(cv::NORM_HAMMING);
//         std::vector<cv::DMatch> matches;
//         matcher.match(descriptors1, descriptors2, matches);

//         // Sort matches based on distance
//         std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
//             return a.distance < b.distance;
//         });

//         // Retain the top matches
//         size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);
//         std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

//         for (const auto& match : goodMatches) {
//             srcPoints.push_back(keypoints1[match.queryIdx].pt);
//             dstPoints.push_back(keypoints2[match.trainIdx].pt);
//         }
//     }

//     // Convert LaserScan data to cv::Mat (Image C)
//     cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
//         int img_size = 500;
//         float max_range = scan->range_max;
//         cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

//         for (size_t i = 0; i < scan->ranges.size(); i++) {
//             float range = scan->ranges[i];
//             if (range > scan->range_min && range < scan->range_max) {
//                 float angle = scan->angle_min + i * scan->angle_increment;
//                 int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
//                 int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
//                 if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
//                     image.at<uchar>(y, x) = 255;
//                 }
//             }
//         }
//         return image;
//     }

//     // Function to extract map section (Image A) around the robot
//     cv::Mat extractMapSectionAroundRobot() {
//         // Placeholder: Replace with actual logic to extract a map section around the robot
//         cv::Mat map_section(500, 500, CV_8UC1, cv::Scalar(0));  // Example: Blank map section
//         // Implement actual map extraction here
//         return map_section;
//     }

//     // ROS node variables
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

//     cv::Mat first_image_, edge_image_, second_image_;
//     bool first_image_captured_ = false;
//     double angle_difference_;
//     double relative_orientation_ = 0.0;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ScanToImageNode>());
//     rclcpp::shutdown();
//     return 0;
// }
