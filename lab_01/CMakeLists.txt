cmake_minimum_required(VERSION 3.8)
project(lab_01)

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

#LabEx01
add_executable(image_processor src/LabEx01_ImageProcessor.cpp)
ament_target_dependencies(image_processor rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  image_processor
  DESTINATION lib/${PROJECT_NAME})


ament_package()
