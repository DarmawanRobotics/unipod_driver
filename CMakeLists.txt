cmake_minimum_required(VERSION 3.5)
project(unipod_driver)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalAttitude.msg"
  "msg/GimbalStatus.msg"
  "msg/CameraStatus.msg"
  "msg/ThermalData.msg"
  "msg/LaserData.msg"
  "msg/AITrackingData.msg"
  "msg/FirmwareVersion.msg"
  "srv/GimbalControl.srv"
  "srv/CameraControl.srv"
  "srv/FocusControl.srv"
  "srv/ZoomControl.srv"
  "srv/SetGimbalAngle.srv"
  "srv/SetVideoMode.srv"
  "srv/ThermalControl.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Library for unipod protocol
add_library(unipod_protocol SHARED
  src/unipod_protocol.cpp
)

ament_target_dependencies(unipod_protocol
  rclcpp
)

rosidl_target_interfaces(unipod_protocol
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Install targets
install(TARGETS
  unipod_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
