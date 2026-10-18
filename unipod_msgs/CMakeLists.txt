cmake_minimum_required(VERSION 3.8)
project(unipod_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
