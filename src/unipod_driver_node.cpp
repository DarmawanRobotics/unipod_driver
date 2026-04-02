#include "unipod_driver/unipod_driver_node.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace unipod_driver {

UnipodDriverNode::UnipodDriverNode(const rclcpp::NodeOptions& options)
    : Node("Unipod_Driver_node", options) {
    
    RCLCPP_INFO(this->get_logger(), "Initializing Unipod Driver Node...");
    
    initializeParameters();
    initializePublishers();
    initializeSubscribers();
    initializeServices();
    
    // Initialize protocol and communication
    protocol_ = std::make_unique<UnipodProtocol>();
    comm_ = std::make_unique<UnipodComm>();
    
    // Connect to gimbal
    if (!connectToGimbal()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to gimbal");
        return;
    }
    
    // Setup receive callback
    comm_->setReceiveCallback(
        std::bind(&UnipodDriverNode::handleReceivedData, this, _1));
    
    // Start heartbeat for TCP
    if (connection_type_ == "tcp") {
        comm_->startHeartbeat(1000);
    }
    
    initializeTimers();
    
    // Request initial status
    auto firmware_packet = protocol_->createRequestFirmwarePacket();
    comm_->sendData(firmware_packet);
    
    // Enable attitude data stream
    auto attitude_stream = protocol_->createRequestDataStreamPacket(1, 4); // 10Hz
    comm_->sendData(attitude_stream);
    
    RCLCPP_INFO(this->get_logger(), "Unipod Driver Node initialized successfully");
}

UnipodDriverNode::~UnipodDriverNode() {
    if (comm_) {
        comm_->disconnect();
    }
}

void UnipodDriverNode::initializeParameters() {
    this->declare_parameter("connection_type", "tcp");
    this->declare_parameter("ip_address", "192.168.144.25");
    this->declare_parameter("port", 37260);
    this->declare_parameter("serial_device", "/dev/ttyUSB0");
    this->declare_parameter("serial_baudrate", 115200);
    this->declare_parameter("attitude_rate", 10);
    this->declare_parameter("status_rate", 1);
    
    connection_type_ = this->get_parameter("connection_type").as_string();
    ip_address_ = this->get_parameter("ip_address").as_string();
    port_ = this->get_parameter("port").as_int();
    serial_device_ = this->get_parameter("serial_device").as_string();
    serial_baudrate_ = this->get_parameter("serial_baudrate").as_int();
    attitude_rate_ = this->get_parameter("attitude_rate").as_int();
    status_rate_ = this->get_parameter("status_rate").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  Connection: %s", connection_type_.c_str());
    if (connection_type_ != "serial") {
        RCLCPP_INFO(this->get_logger(), "  Address: %s:%d", ip_address_.c_str(), port_);
    } else {
        RCLCPP_INFO(this->get_logger(), "  Device: %s @ %d baud", 
                    serial_device_.c_str(), serial_baudrate_);
    }
}

void UnipodDriverNode::initializePublishers() {
    gimbal_attitude_pub_ = this->create_publisher<msg::GimbalAttitude>(
        "~/gimbal_attitude", 10);
    gimbal_status_pub_ = this->create_publisher<msg::GimbalStatus>(
        "~/gimbal_status", 10);
    camera_status_pub_ = this->create_publisher<msg::CameraStatus>(
        "~/camera_status", 10);
    thermal_data_pub_ = this->create_publisher<msg::ThermalData>(
        "~/thermal_data", 10);
    laser_data_pub_ = this->create_publisher<msg::LaserData>(
        "~/laser_data", 10);
    ai_tracking_pub_ = this->create_publisher<msg::AITrackingData>(
        "~/ai_tracking", 10);
    firmware_pub_ = this->create_publisher<msg::FirmwareVersion>(
        "~/firmware_version", 10);
}

void UnipodDriverNode::initializeSubscribers() {
    gimbal_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "~/gimbal_velocity_cmd", 10,
        std::bind(&UnipodDriverNode::gimbalVelocityCallback, this, _1));
}

void UnipodDriverNode::initializeServices() {
    gimbal_control_srv_ = this->create_service<srv::GimbalControl>(
        "~/gimbal_control",
        std::bind(&UnipodDriverNode::handleGimbalControl, this, _1, _2));
    
    camera_control_srv_ = this->create_service<srv::CameraControl>(
        "~/camera_control",
        std::bind(&UnipodDriverNode::handleCameraControl, this, _1, _2));
    
    focus_control_srv_ = this->create_service<srv::FocusControl>(
        "~/focus_control",
        std::bind(&UnipodDriverNode::handleFocusControl, this, _1, _2));
    
    zoom_control_srv_ = this->create_service<srv::ZoomControl>(
        "~/zoom_control",
        std::bind(&UnipodDriverNode::handleZoomControl, this, _1, _2));
    
    set_angle_srv_ = this->create_service<srv::SetGimbalAngle>(
        "~/set_gimbal_angle",
        std::bind(&UnipodDriverNode::handleSetGimbalAngle, this, _1, _2));
    
    set_video_mode_srv_ = this->create_service<srv::SetVideoMode>(
        "~/set_video_mode",
        std::bind(&UnipodDriverNode::handleSetVideoMode, this, _1, _2));
    
    thermal_control_srv_ = this->create_service<srv::ThermalControl>(
        "~/thermal_control",
        std::bind(&UnipodDriverNode::handleThermalControl, this, _1, _2));
    
    recenter_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/recenter_gimbal",
        std::bind(&UnipodDriverNode::handleRecenterGimbal, this, _1, _2));
}

void UnipodDriverNode::initializeTimers() {
    if (status_rate_ > 0) {
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / status_rate_),
            std::bind(&UnipodDriverNode::statusUpdateTimer, this));
    }
}

bool UnipodDriverNode::connectToGimbal() {
    bool success = false;
    
    if (connection_type_ == "tcp") {
        success = comm_->connectTCP(ip_address_, port_);
    } else if (connection_type_ == "udp") {
        success = comm_->connectUDP(ip_address_, port_);
    } else if (connection_type_ == "serial") {
        success = comm_->connectSerial(serial_device_, serial_baudrate_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid connection type: %s", 
                     connection_type_.c_str());
        return false;
    }
    
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Connected to Unipod gimbal");
    }
    
    return success;
}

void UnipodDriverNode::handleReceivedData(const std::vector<uint8_t>& data) {
    // Verify CRC
    if (!protocol_->verifyCRC(data)) {
        RCLCPP_WARN(this->get_logger(), "CRC verification failed");
        return;
    }
    
    // Parse packet
    PacketHeader header;
    std::vector<uint8_t> payload;
    
    if (!protocol_->parsePacket(data, header, payload)) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse packet");
        return;
    }
    
    // Process the packet
    processPacket(header, payload);
}

void UnipodDriverNode::processPacket(const PacketHeader& header, 
                                 const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    switch (header.cmd_id) {
        case CMD_FIRMWARE_VERSION: {
            if (data.size() >= 12) {
                msg::FirmwareVersion msg;
                msg.header.stamp = this->now();
                msg.camera_firmware_version = data[0] | (data[1] << 8) | 
                                             (data[2] << 16) | (data[3] << 24);
                msg.gimbal_firmware_version = data[4] | (data[5] << 8) | 
                                             (data[6] << 16) | (data[7] << 24);
                msg.zoom_firmware_version = data[8] | (data[9] << 8) | 
                                           (data[10] << 16) | (data[11] << 24);
                firmware_pub_->publish(msg);
            }
            break;
        }
        
        case CMD_GIMBAL_ATTITUDE: {
            if (data.size() >= 12) {
                int16_t yaw = data[0] | (data[1] << 8);
                int16_t pitch = data[2] | (data[3] << 8);
                int16_t roll = data[4] | (data[5] << 8);
                int16_t yaw_vel = data[6] | (data[7] << 8);
                int16_t pitch_vel = data[8] | (data[9] << 8);
                int16_t roll_vel = data[10] | (data[11] << 8);
                
                current_attitude_.header.stamp = this->now();
                current_attitude_.yaw = yaw / 10.0f;
                current_attitude_.pitch = pitch / 10.0f;
                current_attitude_.roll = roll / 10.0f;
                current_attitude_.yaw_velocity = yaw_vel / 10.0f;
                current_attitude_.pitch_velocity = pitch_vel / 10.0f;
                current_attitude_.roll_velocity = roll_vel / 10.0f;
                
                gimbal_attitude_pub_->publish(current_attitude_);
            }
            break;
        }
        
        case CMD_LASER_RANGING: {
            if (data.size() >= 2) {
                uint16_t distance_dm = data[0] | (data[1] << 8);
                
                msg::LaserData msg;
                msg.header.stamp = this->now();
                msg.distance = (distance_dm >= 50) ? (distance_dm / 10.0f) : 0.0f;
                laser_data_pub_->publish(msg);
            }
            break;
        }
        
        case CMD_AI_TRACKING_DATA: {
            if (data.size() >= 10) {
                msg::AITrackingData msg;
                msg.header.stamp = this->now();
                msg.pos_x = data[0] | (data[1] << 8);
                msg.pos_y = data[2] | (data[3] << 8);
                msg.width = data[4] | (data[5] << 8);
                msg.height = data[6] | (data[7] << 8);
                msg.target_id = data[8];
                msg.tracking_status = data[9];
                ai_tracking_pub_->publish(msg);
            }
            break;
        }
        
        case CMD_CAMERA_INFO: {
            if (data.size() >= 8) {
                current_camera_status_.header.stamp = this->now();
                current_camera_status_.hdr_enabled = (data[1] == 1);
                current_camera_status_.recording = (data[3] == 1);
                current_camera_status_.tf_card_present = (data[3] != 2);
                current_status_.motion_mode = data[4];
                current_status_.mounting_direction = data[5];
                current_camera_status_.video_output_mode = data[6];
                current_camera_status_.zoom_linkage_enabled = (data[7] == 1);
                
                camera_status_pub_->publish(current_camera_status_);
            }
            break;
        }
        
        default:
            // Handle other command responses
            break;
    }
}

// Service handlers implementation continues...
void UnipodDriverNode::handleGimbalControl(
    const std::shared_ptr<srv::GimbalControl::Request> request,
    std::shared_ptr<srv::GimbalControl::Response> response) {
    
    auto packet = protocol_->createGimbalRotationPacket(
        request->yaw_speed, request->pitch_speed);
    
    response->success = comm_->sendData(packet);
    response->message = response->success ? "Command sent" : "Failed to send command";
}

void UnipodDriverNode::handleCameraControl(
    const std::shared_ptr<srv::CameraControl::Request> request,
    std::shared_ptr<srv::CameraControl::Response> response) {
    
    auto packet = protocol_->createPhotoVideoPacket(request->command);
    response->success = comm_->sendData(packet);
    response->message = response->success ? "Command sent" : "Failed to send command";
}

void UnipodDriverNode::handleFocusControl(
    const std::shared_ptr<srv::FocusControl::Request> request,
    std::shared_ptr<srv::FocusControl::Response> response) {
    
    std::vector<uint8_t> data(5);
    data[0] = request->auto_focus ? 1 : 0;
    data[1] = request->touch_x & 0xFF;
    data[2] = (request->touch_x >> 8) & 0xFF;
    data[3] = request->touch_y & 0xFF;
    data[4] = (request->touch_y >> 8) & 0xFF;
    
    auto packet = protocol_->createPacket(CMD_AUTO_FOCUS, data, true);
    response->success = comm_->sendData(packet);
    response->message = response->success ? "Focus command sent" : "Failed to send command";
}

void UnipodDriverNode::handleZoomControl(
    const std::shared_ptr<srv::ZoomControl::Request> request,
    std::shared_ptr<srv::ZoomControl::Response> response) {
    
    std::vector<uint8_t> packet;
    
    if (request->absolute_zoom > 0.0f) {
        uint8_t zoom_int = static_cast<uint8_t>(request->absolute_zoom);
        uint8_t zoom_float = static_cast<uint8_t>((request->absolute_zoom - zoom_int) * 10);
        packet = protocol_->createAbsoluteZoomPacket(zoom_int, zoom_float);
    } else {
        packet = protocol_->createZoomPacket(request->zoom_command);
    }
    
    response->success = comm_->sendData(packet);
    response->current_zoom = 0.0f; // Will be updated by response
    response->message = response->success ? "Zoom command sent" : "Failed to send command";
}

void UnipodDriverNode::handleSetGimbalAngle(
    const std::shared_ptr<srv::SetGimbalAngle::Request> request,
    std::shared_ptr<srv::SetGimbalAngle::Response> response) {
    
    int16_t yaw = static_cast<int16_t>(request->yaw * 10.0f);
    int16_t pitch = static_cast<int16_t>(request->pitch * 10.0f);
    
    auto packet = protocol_->createSetGimbalAnglePacket(yaw, pitch);
    response->success = comm_->sendData(packet);
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    response->current_yaw = current_attitude_.yaw;
    response->current_pitch = current_attitude_.pitch;
    response->current_roll = current_attitude_.roll;
    response->message = response->success ? "Angle command sent" : "Failed to send command";
}

void UnipodDriverNode::handleSetVideoMode(
    const std::shared_ptr<srv::SetVideoMode::Request> request,
    std::shared_ptr<srv::SetVideoMode::Response> response) {
    
    std::vector<uint8_t> data = {request->main_stream, request->sub_stream};
    auto packet = protocol_->createPacket(CMD_SET_VIDEO_MODE, data, true);
    
    response->success = comm_->sendData(packet);
    response->current_main_stream = request->main_stream;
    response->current_sub_stream = request->sub_stream;
    response->message = response->success ? "Video mode set" : "Failed to send command";
}

void UnipodDriverNode::handleThermalControl(
    const std::shared_ptr<srv::ThermalControl::Request> request,
    std::shared_ptr<srv::ThermalControl::Response> response) {
    
    std::vector<uint8_t> packet;
    
    switch (request->command) {
        case 0: { // Set palette
            std::vector<uint8_t> data = {request->palette_mode};
            packet = protocol_->createPacket(CMD_SET_THERMAL_PALETTE, data, true);
            break;
        }
        case 1: { // Set gain
            std::vector<uint8_t> data = {request->gain_mode};
            packet = protocol_->createPacket(CMD_SET_THERMAL_GAIN, data, true);
            break;
        }
        case 4: { // Update shutter
            packet = protocol_->createPacket(CMD_UPDATE_THERMAL_SHUTTER, {}, true);
            break;
        }
        default:
            response->success = false;
            response->message = "Unknown thermal command";
            return;
    }
    
    response->success = comm_->sendData(packet);
    response->temperature = 0.0f;
    response->message = response->success ? "Thermal command sent" : "Failed to send command";
}

void UnipodDriverNode::handleRecenterGimbal(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    std::vector<uint8_t> data = {1}; // One-key reset
    auto packet = protocol_->createPacket(CMD_GIMBAL_CENTER, data, true);
    
    response->success = comm_->sendData(packet);
    response->message = response->success ? "Gimbal recentered" : "Failed to send command";
}

void UnipodDriverNode::gimbalVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Map twist to gimbal speeds (-100 to 100)
    int8_t yaw_speed = static_cast<int8_t>(std::max(-100.0, std::min(100.0, msg->angular.z * 100.0)));
    int8_t pitch_speed = static_cast<int8_t>(std::max(-100.0, std::min(100.0, msg->angular.y * 100.0)));
    
    auto packet = protocol_->createGimbalRotationPacket(yaw_speed, pitch_speed);
    comm_->sendData(packet);
}

void UnipodDriverNode::statusUpdateTimer() {
    // Request camera system info
    auto info_packet = protocol_->createPacket(CMD_CAMERA_INFO, {}, true);
    comm_->sendData(info_packet);
    
    // Publish current status
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_status_.header.stamp = this->now();
    gimbal_status_pub_->publish(current_status_);
}

} // namespace unipod_driver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<unipod_driver::UnipodDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
