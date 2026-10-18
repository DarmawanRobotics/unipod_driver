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
find_package(unipod_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

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
target_link_libraries(unipod_driver_node unipod_client)
ament_target_dependencies(unipod_driver_node rclcpp std_msgs geometry_msgs unipod_msgs)

# RTSP capture (OpenCV, no ROS) and the camera node: one per stream, video1 / video2
add_library(unipod_rtsp SHARED src/rtsp_capture.cpp)
target_include_directories(unipod_rtsp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(unipod_rtsp SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(unipod_rtsp ${OpenCV_LIBS} Threads::Threads)

add_executable(unipod_camera_node src/unipod_camera_node.cpp)
target_link_libraries(unipod_camera_node unipod_rtsp)
ament_target_dependencies(unipod_camera_node rclcpp cv_bridge image_transport)

install(TARGETS unipod_client unipod_rtsp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS unipod_driver_node unipod_camera_node DESTINATION lib/${PROJECT_NAME})
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
  ament_add_gtest(test_rtsp test/test_rtsp.cpp TIMEOUT 60)
  target_link_libraries(test_rtsp unipod_rtsp)
endif()

ament_package()
