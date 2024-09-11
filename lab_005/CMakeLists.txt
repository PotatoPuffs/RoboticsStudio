cmake_minimum_required(VERSION 3.8)
project(lab_005)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(scanning src/scan_to_image.cpp)
ament_target_dependencies(scanning rclcpp sensor_msgs geometry_msgs nav_msgs OpenCV cv_bridge)

install(TARGETS
  scanning
  DESTINATION lib/${PROJECT_NAME})

ament_package()