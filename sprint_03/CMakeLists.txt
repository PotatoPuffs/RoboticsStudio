cmake_minimum_required(VERSION 3.8)
project(sprint_03)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs)


add_executable(test1 src/test1.cpp)
ament_target_dependencies(test1 rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_geometry_msgs)

add_executable(test2 src/test2.cpp)
ament_target_dependencies(test2 rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  test1
  test2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
