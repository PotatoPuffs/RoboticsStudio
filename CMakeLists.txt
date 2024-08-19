cmake_minimum_required(VERSION 3.8)
project(RoboticsStudio)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

#LabEx01
add_executable(image_processor LabExercises/LabEx01_ImageProcessor.cpp)
ament_target_dependencies(image_processor rclcpp sensor_msgs cv_bridge OpenCV)

#LabEx02
add_executable(laser_scanner LabExercises/LabEx02_Scanner.cpp)
ament_target_dependencies(laser_scanner rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

#Sprint1
add_executable(sprint_one Sprints/Sprint1_Task.cpp)
ament_target_dependencies(sprint_one rclcpp sensor_msgs std_msgs)

install(TARGETS
  image_processor
  laser_scanner
  sprint_one
  DESTINATION lib/${PROJECT_NAME})


ament_package()
