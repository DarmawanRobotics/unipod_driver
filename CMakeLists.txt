cmake_minimum_required(VERSION 3.8)
project(unipod_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalAttitude.msg"
  "msg/GimbalStatus.msg"
  "msg/ThermalData.msg"
  "srv/GetTemperature.srv"
  "srv/Recenter.srv"
  "srv/SetGimbalAngle.srv"
  "srv/SetVideoMode.srv"
  "srv/SetZoom.srv"
  "srv/ThermalControl.srv"
  "srv/ZoomControl.srv"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ROS-free client: protocol, transports, request/ACK handling
add_library(unipod_client SHARED
  src/protocol.cpp
  src/transport.cpp
  src/client.cpp
)
target_include_directories(unipod_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(unipod_client Threads::Threads)

# Driver node: PTZ, zoom, video streams, thermal, recenter
add_executable(unipod_driver_node src/unipod_driver_node.cpp)
target_link_libraries(unipod_driver_node unipod_client "${cpp_typesupport_target}")
ament_target_dependencies(unipod_driver_node rclcpp std_msgs geometry_msgs)

install(TARGETS unipod_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS unipod_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol unipod_client)
  ament_add_gtest(test_client test/test_client.cpp)
  target_link_libraries(test_client unipod_client)
  ament_add_gtest(test_transport test/test_transport.cpp)
  target_link_libraries(test_transport unipod_client)
  ament_add_gtest(test_types test/test_types.cpp)
  target_include_directories(test_types PRIVATE include)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
