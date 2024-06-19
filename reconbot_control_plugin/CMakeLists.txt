cmake_minimum_required(VERSION 3.8)
project(reconbot_control_plugin)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# Ensure the C++ standard is set to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

add_compile_options(-std=c++14)

include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(reconbot_control_interface_plugin SHARED src/reconbot_control_interface_plugin.cpp)
ament_target_dependencies(reconbot_control_interface_plugin rclcpp std_msgs gazebo_ros)

install(TARGETS reconbot_control_interface_plugin
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include/)

install(PROGRAMS scripts/reconbot_control.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
