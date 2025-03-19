cmake_minimum_required(VERSION 3.8)
project(masbot_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Hardware interface library
add_library(
  masbot_hardware_interface
  SHARED
  src/masbot_hardware_interface.cpp
)

target_include_directories(
  masbot_hardware_interface
  PRIVATE
  include
)

ament_target_dependencies(
  masbot_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
)

# Export hardware interface as a plugin
pluginlib_export_plugin_description_file(
  hardware_interface
  masbot_hardware_interface.xml
)

# Install directories
install(
  DIRECTORY urdf launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install hardware interface library
install(
  TARGETS masbot_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install plugin description file
install(
  FILES masbot_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(masbot_hardware_interface)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_components
  controller_interface
)

ament_package()
