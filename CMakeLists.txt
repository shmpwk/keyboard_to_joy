cmake_minimum_required(VERSION 3.5)
project(keyboard_to_joy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

ament_auto_find_build_dependencies()

#ament_auto_add_executable(keyboard_to_joy
#  src/keyboard_to_joy.cpp
#)
ament_auto_add_library(keyboard_to_joy_node SHARED
  src/keyboard_to_joy.cpp
)
rclcpp_components_register_node(keyboard_to_joy_node
  PLUGIN "Keyboard2Joy"
  EXECUTABLE keyboard_to_joy
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)