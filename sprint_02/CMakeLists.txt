cmake_minimum_required(VERSION 3.8)
project(sprint_02)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(sprint_two src/sprint_two.cpp)
ament_target_dependencies(sprint_two rclcpp sensor_msgs geometry_msgs nav_msgs OpenCV cv_bridge tf2 tf2_geometry_msgs)

add_executable(test1 src/test1.cpp)
ament_target_dependencies(test1 rclcpp sensor_msgs geometry_msgs nav_msgs OpenCV cv_bridge tf2 tf2_geometry_msgs)

add_executable(test2 src/test2.cpp)
ament_target_dependencies(test2 rclcpp sensor_msgs geometry_msgs nav_msgs OpenCV cv_bridge tf2 tf2_geometry_msgs)

add_executable(test3 src/test3.cpp)
ament_target_dependencies(test3 rclcpp sensor_msgs geometry_msgs nav_msgs OpenCV cv_bridge tf2 tf2_geometry_msgs)

add_executable(test4 src/test4.cpp)
ament_target_dependencies(test4 rclcpp sensor_msgs geometry_msgs nav_msgs OpenCV cv_bridge tf2 tf2_geometry_msgs)

install(TARGETS
  sprint_two
  test1
  test2
  test3
  test4
  DESTINATION lib/${PROJECT_NAME})


ament_package()
