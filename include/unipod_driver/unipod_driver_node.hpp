#ifndef UNIPOD_DRIVER_NODE_HPP
#define UNIPOD_DRIVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "unipod_driver/unipod_protocol.hpp"
#include "unipod_driver/unipod_comm.hpp"

// Custom messages
#include "unipod_driver/msg/gimbal_attitude.hpp"
#include "unipod_driver/msg/gimbal_status.hpp"
#include "unipod_driver/msg/camera_status.hpp"
#include "unipod_driver/msg/thermal_data.hpp"
#include "unipod_driver/msg/laser_data.hpp"
#include "unipod_driver/msg/ai_tracking_data.hpp"
#include "unipod_driver/msg/firmware_version.hpp"

// Custom services
#include "unipod_driver/srv/gimbal_control.hpp"
#include "unipod_driver/srv/camera_control.hpp"
#include "unipod_driver/srv/focus_control.hpp"
#include "unipod_driver/srv/zoom_control.hpp"
#include "unipod_driver/srv/set_gimbal_angle.hpp"
#include "unipod_driver/srv/set_video_mode.hpp"
#include "unipod_driver/srv/thermal_control.hpp"

namespace unipod_driver {

class UnipodDriverNode : public rclcpp::Node {
public:
    explicit UnipodDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~UnipodDriverNode();

private:
    // Initialization
    void initializeParameters();
    void initializePublishers();
    void initializeSubscribers();
    void initializeServices();
    void initializeTimers();
    
    // Connection
    bool connectToGimbal();
    
    // Data callbacks
    void handleReceivedData(const std::vector<uint8_t>& data);
    void processPacket(const PacketHeader& header, const std::vector<uint8_t>& data);
    
    // Publishers callbacks
    void publishGimbalAttitude();
    void publishGimbalStatus();
    void publishCameraStatus();
    
    // Service callbacks
    void handleGimbalControl(
        const std::shared_ptr<srv::GimbalControl::Request> request,
        std::shared_ptr<srv::GimbalControl::Response> response);
    
    void handleCameraControl(
        const std::shared_ptr<srv::CameraControl::Request> request,
        std::shared_ptr<srv::CameraControl::Response> response);
    
    void handleFocusControl(
        const std::shared_ptr<srv::FocusControl::Request> request,
        std::shared_ptr<srv::FocusControl::Response> response);
    
    void handleZoomControl(
        const std::shared_ptr<srv::ZoomControl::Request> request,
        std::shared_ptr<srv::ZoomControl::Response> response);
    
    void handleSetGimbalAngle(
        const std::shared_ptr<srv::SetGimbalAngle::Request> request,
        std::shared_ptr<srv::SetGimbalAngle::Response> response);
    
    void handleSetVideoMode(
        const std::shared_ptr<srv::SetVideoMode::Request> request,
        std::shared_ptr<srv::SetVideoMode::Response> response);
    
    void handleThermalControl(
        const std::shared_ptr<srv::ThermalControl::Request> request,
        std::shared_ptr<srv::ThermalControl::Response> response);
    
    void handleRecenterGimbal(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // Subscriber callbacks
    void gimbalVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    // Timer callbacks
    void statusUpdateTimer();
    void requestAttitudeData();
    
    // Protocol and communication
    std::unique_ptr<UnipodProtocol> protocol_;
    std::unique_ptr<UnipodComm> comm_;
    
    // Publishers
    rclcpp::Publisher<msg::GimbalAttitude>::SharedPtr gimbal_attitude_pub_;
    rclcpp::Publisher<msg::GimbalStatus>::SharedPtr gimbal_status_pub_;
    rclcpp::Publisher<msg::CameraStatus>::SharedPtr camera_status_pub_;
    rclcpp::Publisher<msg::ThermalData>::SharedPtr thermal_data_pub_;
    rclcpp::Publisher<msg::LaserData>::SharedPtr laser_data_pub_;
    rclcpp::Publisher<msg::AITrackingData>::SharedPtr ai_tracking_pub_;
    rclcpp::Publisher<msg::FirmwareVersion>::SharedPtr firmware_pub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gimbal_velocity_sub_;
    
    // Services
    rclcpp::Service<srv::GimbalControl>::SharedPtr gimbal_control_srv_;
    rclcpp::Service<srv::CameraControl>::SharedPtr camera_control_srv_;
    rclcpp::Service<srv::FocusControl>::SharedPtr focus_control_srv_;
    rclcpp::Service<srv::ZoomControl>::SharedPtr zoom_control_srv_;
    rclcpp::Service<srv::SetGimbalAngle>::SharedPtr set_angle_srv_;
    rclcpp::Service<srv::SetVideoMode>::SharedPtr set_video_mode_srv_;
    rclcpp::Service<srv::ThermalControl>::SharedPtr thermal_control_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recenter_srv_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr attitude_timer_;
    
    // Parameters
    std::string connection_type_;
    std::string ip_address_;
    int port_;
    std::string serial_device_;
    int serial_baudrate_;
    int attitude_rate_;
    int status_rate_;
    
    // State variables
    msg::GimbalAttitude current_attitude_;
    msg::GimbalStatus current_status_;
    msg::CameraStatus current_camera_status_;
    std::mutex state_mutex_;
};

} // namespace unipod_driver

#endif // UNIPOD_DRIVER_NODE_HPP
