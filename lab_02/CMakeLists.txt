cmake_minimum_required(VERSION 3.8)
project(lab_02)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)


#LabEx02
add_executable(laser_scanner src/LabEx02_Scanner.cpp)
ament_target_dependencies(laser_scanner rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

install(TARGETS
  laser_scanner
  DESTINATION lib/${PROJECT_NAME})


ament_package()
