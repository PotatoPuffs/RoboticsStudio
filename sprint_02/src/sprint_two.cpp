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
//         // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
//         cv::Mat img = laserScanToMat(msg);

//         if (!first_image_captured_) {
//             // Step 2: Extract section of the map around the robot (Image A)
//             first_image_ = img.clone();
//             first_image_captured_ = true;
//             // Display the first image
//             cv::imshow("First Image", first_image_);
//             cv::waitKey(1);  // Add this to process GUI events and update the window
//             // Step 7: Propagate robot to a new location
//             rotateRobot(); // Uncomment to rotate the robot
//         } else if (!second_image_captured_) {
//             // Step 3: Create an edge image (Image B) from Image A (This could be added to the process, not yet implemented)
//             second_image_ = img.clone();
//             second_image_captured_ = true;
//             // Display the second image
//             cv::imshow("Second Image", second_image_);
//             cv::waitKey(1);  // Add this to process GUI events and update the window
//             // Step 4: Receive laser scan and convert it to an image (Image C)
//         } else {
//             // Step 4: Receive laser scan and convert it to an image (Image C)
//             first_image_ = second_image_.clone();
//             second_image_ = img.clone();
//             // Display the new second image
//             cv::imshow("Second Image", second_image_);
//             cv::waitKey(1);  // Add this to process GUI events and update the window

//             // Step 5: Use Image B and Image C to estimate rotation
//             calculateYawChange();
//             relative_orientaion_ += angle_difference_;
//             RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f", relative_orientaion_);
//         }
//     }

//     cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
//         // Convert LaserScan data to cv::Mat (Image C)
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

//     void calculateYawChange() {
//         // Step 5: Detect and match features between Image B and Image C
//         std::vector<cv::Point2f> srcPoints, dstPoints;
//         detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

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
//                 angle_difference_ = angle_difference_ * 180.0 / CV_PI;
//                 RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
//             }
//         } catch (const cv::Exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
//         }
//     }

//     void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
//                                 std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
//         // Detect and match features between two images
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

//         for (const auto& match : goodMatches) {
//             srcPoints.push_back(keypoints1[match.queryIdx].pt);
//             dstPoints.push_back(keypoints2[match.trainIdx].pt);
//         }
//     }

//     // Optional: Rotate robot
//     void rotateRobot() {
//         auto twist_msg = geometry_msgs::msg::Twist();
//         twist_msg.angular.z = 1.0;  // Rotate with some angular velocity
//         cmd_publisher_->publish(twist_msg);

//         // Sleep for a while to allow the robot to rotate
//         rclcpp::sleep_for(std::chrono::seconds(2));

//         // Stop rotation
//         twist_msg.angular.z = 0.0;
//         cmd_publisher_->publish(twist_msg);
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


// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <iostream>

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
//         // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
//         cv::Mat img = laserScanToMat(msg);

//         if (!first_image_captured_) {
//             // Step 2: Extract section of the map around the robot (Image A)
//             first_image_ = img.clone();
//             first_image_captured_ = true;
//             // Create edge image from first image
//             edge_image_ = createEdgeImage(first_image_);
//             // Display the first image and edge image
//             cv::imshow("First Image", first_image_);
//             cv::imshow("Edge Image", edge_image_);
//             cv::waitKey(1);  // Add this to process GUI events and update the window
//             // Step 7: Propagate robot to a new location
//             rotateRobot(); // Uncomment to rotate the robot
//         } else if (!second_image_captured_) {
//             // Step 3: Create an edge image (Image B) from the second image
//             second_image_ = img.clone();
//             edge_image_ = createEdgeImage(second_image_);
//             second_image_captured_ = true;
//             // Display the second image and edge image
//             cv::imshow("Second Image", second_image_);
//             cv::imshow("Edge Image", edge_image_);
//             cv::waitKey(1);  // Add this to process GUI events and update the window
//         } else {
//             // Step 4: Receive laser scan and convert it to an image (Image C)
//             first_image_ = second_image_.clone();
//             second_image_ = img.clone();
//             edge_image_ = createEdgeImage(second_image_);
//             // Display the new second image and edge image
//             cv::imshow("Second Image", second_image_);
//             cv::imshow("Edge Image", edge_image_);
//             cv::waitKey(1);  // Add this to process GUI events and update the window

//             // Step 5: Use Image B and Image C to estimate rotation
//             calculateYawChange();
//             relative_orientaion_ += angle_difference_;
//             RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f", relative_orientaion_);
//         }
//     }

//     cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
//         // Convert LaserScan data to cv::Mat (Image C)
//         int img_size = 500;
//         float max_range = scan->range_max;
//         cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

//                 for (size_t i = 0; i < scan->ranges.size(); i++) {
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

//     cv::Mat createEdgeImage(const cv::Mat& input_image) {
//         cv::Mat edge_image;
//         cv::Canny(input_image, edge_image, 50, 150);  // Adjust thresholds as needed
//         return edge_image;
//     }

//     void calculateYawChange() {
//         // Step 5: Detect and match features between Image B and Image C
//         std::vector<cv::Point2f> srcPoints, dstPoints;
//         detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

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
//                 angle_difference_ = angle_difference_ * 180.0 / CV_PI;
//                 RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
//             }
//         } catch (const cv::Exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
//         }
//     }

//     void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
//                                 std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
//         // Detect and match features between two images
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

//         for (const auto& match : goodMatches) {
//             srcPoints.push_back(keypoints1[match.queryIdx].pt);
//             dstPoints.push_back(keypoints2[match.trainIdx].pt);
//         }
//     }

//     // Optional: Rotate robot
//     void rotateRobot() {
//         auto twist_msg = geometry_msgs::msg::Twist();
//         twist_msg.angular.z = 1.0;  // Rotate with some angular velocity
//         cmd_publisher_->publish(twist_msg);

//         // Sleep for a while to allow the robot to rotate
//         rclcpp::sleep_for(std::chrono::seconds(2));

//         // Stop rotation
//         twist_msg.angular.z = 0.0;
//         cmd_publisher_->publish(twist_msg);
//     }

//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

//     cv::Mat first_image_, second_image_, edge_image_;
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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>

class ScanToImageNode : public rclcpp::Node {
public:
    ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientaion_(0.0), target_yaw_(0.0) {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
        cv::Mat scan_image = laserScanToMat(msg);

        if (!first_image_captured_) {
            // Capture the map image (Image A) as the first image
            map_image_ = scan_image.clone();
            first_image_captured_ = true;
            RCLCPP_INFO(this->get_logger(), "Map image captured.");
            cv::imshow("Map Image", map_image_);
            cv::waitKey(1);  // Process GUI events and update the window
        } else {
            // Compare the scan image (Image B) to the map image (Image A) and calculate yaw
            double estimated_yaw = compareImagesAndEstimateYaw(map_image_, scan_image);

            if (fabs(estimated_yaw) > 0.1) {
                // Adjust the robot's orientation to correct the yaw
                rotateRobot();
                RCLCPP_INFO(this->get_logger(), "Adjusting yaw by: %f degrees", estimated_yaw);
                // Display the current scan image and adjusted map image
                cv::imshow("Current Scan Image", scan_image);
                cv::waitKey(1);  // Process GUI events and update the window
            } else {
                // Stop rotation if yaw is close to 0
                auto twist_msg = geometry_msgs::msg::Twist();
                twist_msg.angular.z = 0.0;
                cmd_publisher_->publish(twist_msg);
                RCLCPP_INFO(this->get_logger(), "Yaw is close to 0. Stopping.");
                cv::imshow("Current Scan Image", scan_image);
                cv::waitKey(1);  // Display the final scan image
                
            }
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

    double compareImagesAndEstimateYaw(const cv::Mat& map_image, const cv::Mat& scan_image) {
        // Use feature matching to estimate yaw change
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(map_image, scan_image, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return 0.0;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
                return 0.0;
            } else {
                // Extract the rotation angle from the transformation matrix
                double angle = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                double angle_deg = angle * 180.0 / CV_PI;
                return angle_deg;
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
            return 0.0;
        }
    }

    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        // Detect and match features between two images
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
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
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : goodMatches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }
  
     void rotateRobot() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 1.0;  // Rotate with some angular velocity
        cmd_publisher_->publish(twist_msg);

        // Sleep for a while to allow the robot to rotate
        rclcpp::sleep_for(std::chrono::seconds(2));

        // Stop rotation
        twist_msg.angular.z = 0.0;
        cmd_publisher_->publish(twist_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat map_image_;
    bool first_image_captured_ = false;
    double angle_difference_;
    double relative_orientaion_;
    double target_yaw_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToImageNode>());
    rclcpp::shutdown();
    return 0;
}


