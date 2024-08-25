cmake_minimum_required(VERSION 3.8)
project(sprint_01)

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

#Sprint1
add_executable(sprint_one src/Sprint1_Task.cpp)
ament_target_dependencies(sprint_one rclcpp sensor_msgs std_msgs)

install(TARGETS
  sprint_one
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME})


ament_package()
