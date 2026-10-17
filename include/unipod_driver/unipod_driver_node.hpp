// ROS 2 node for the UniPod MT11: PTZ, zoom, video streams, thermal, recenter.
#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "unipod_driver/client.hpp"
#include "unipod_driver/msg/gimbal_attitude.hpp"
#include "unipod_driver/msg/gimbal_status.hpp"
#include "unipod_driver/msg/thermal_data.hpp"
#include "unipod_driver/srv/get_temperature.hpp"
#include "unipod_driver/srv/recenter.hpp"
#include "unipod_driver/srv/set_gimbal_angle.hpp"
#include "unipod_driver/srv/set_video_mode.hpp"
#include "unipod_driver/srv/set_zoom.hpp"
#include "unipod_driver/srv/thermal_control.hpp"
#include "unipod_driver/srv/zoom_control.hpp"

namespace unipod_driver
{

class UnipodDriverNode : public rclcpp::Node
{
public:
  explicit UnipodDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UnipodDriverNode() override;

private:
  void declareParameters();
  void onLinkUp();
  void statusTick();
  void ptzWatchdog();
  void onPtzCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void sendPtz(int8_t yaw, int8_t pitch, int8_t zoom);
  msg::ThermalData toMsg(const Temperature & t);

  // Parameters
  std::string frame_id_;
  std::string thermal_frame_id_;
  int ptz_max_speed_{100};
  bool invert_yaw_{false};
  bool invert_pitch_{false};
  double ptz_timeout_sec_{0.5};
  StreamRate attitude_rate_{kRate10Hz};
  int main_stream_{-1};
  int sub_stream_{-1};
  int thermal_palette_{-1};
  int thermal_gain_{-1};
  bool temperature_stream_{true};

  std::unique_ptr<UnipodClient> client_;

  rclcpp::Publisher<msg::GimbalAttitude>::SharedPtr attitude_pub_;
  rclcpp::Publisher<msg::GimbalStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<msg::ThermalData>::SharedPtr temperature_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ptz_sub_;

  rclcpp::Service<srv::SetGimbalAngle>::SharedPtr set_angle_srv_;
  rclcpp::Service<srv::Recenter>::SharedPtr recenter_srv_;
  rclcpp::Service<srv::ZoomControl>::SharedPtr zoom_srv_;
  rclcpp::Service<srv::SetZoom>::SharedPtr set_zoom_srv_;
  rclcpp::Service<srv::SetVideoMode>::SharedPtr video_mode_srv_;
  rclcpp::Service<srv::GetTemperature>::SharedPtr temperature_srv_;
  rclcpp::Service<srv::ThermalControl>::SharedPtr thermal_srv_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr ptz_timer_;

  // Blocking requests (services, status poll) and PTZ run in separate groups, so a slow
  // service call never delays a stop command.
  rclcpp::CallbackGroup::SharedPtr request_group_;
  rclcpp::CallbackGroup::SharedPtr ptz_group_;

  std::mutex state_mtx_;
  msg::GimbalStatus status_;
  bool was_up_{false};

  std::mutex ptz_mtx_;
  int8_t last_yaw_{0}, last_pitch_{0}, last_zoom_{0};
  std::chrono::steady_clock::time_point last_ptz_cmd_{};
};

}  // namespace unipod_driver
