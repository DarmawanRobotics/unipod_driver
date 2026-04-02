#ifndef Unipod_PROTOCOL_HPP
#define Unipod_PROTOCOL_HPP

#include <cstdint>
#include <vector>
#include <cstring>

namespace unipod_driver {

// Protocol constants
const uint16_t PROTOCOL_STX = 0x6655;
const uint8_t PROTOCOL_HEADER_SIZE = 8;
const uint8_t PROTOCOL_CRC_SIZE = 2;

// Command IDs
enum CommandID {
    CMD_TCP_HEARTBEAT = 0x00,
    CMD_FIRMWARE_VERSION = 0x01,
    CMD_HARDWARE_ID = 0x02,
    CMD_AUTO_FOCUS = 0x04,
    CMD_MANUAL_ZOOM = 0x05,
    CMD_MANUAL_FOCUS = 0x06,
    CMD_GIMBAL_ROTATION = 0x07,
    CMD_GIMBAL_CENTER = 0x08,
    CMD_CAMERA_INFO = 0x0A,
    CMD_TRANSMISSION_FEEDBACK = 0x0B,
    CMD_PHOTO_VIDEO = 0x0C,
    CMD_GIMBAL_ATTITUDE = 0x0D,
    CMD_SET_GIMBAL_ANGLE = 0x0E,
    CMD_ABSOLUTE_ZOOM = 0x0F,
    CMD_GET_VIDEO_MODE = 0x10,
    CMD_SET_VIDEO_MODE = 0x11,
    CMD_GET_TEMP_POINT = 0x12,
    CMD_GET_TEMP_REGION = 0x13,
    CMD_GET_TEMP_FULLFRAME = 0x14,
    CMD_LASER_RANGING = 0x15,
    CMD_GET_ZOOM_RANGE = 0x16,
    CMD_LASER_COORDINATES = 0x17,
    CMD_GET_ZOOM_LEVEL = 0x18,
    CMD_GET_GIMBAL_MODE = 0x19,
    CMD_GET_THERMAL_PALETTE = 0x1A,
    CMD_SET_THERMAL_PALETTE = 0x1B,
    CMD_GET_ENCODING_PARAMS = 0x20,
    CMD_SET_ENCODING_PARAMS = 0x21,
    CMD_SEND_ATTITUDE = 0x22,
    CMD_REQUEST_DATA_STREAM = 0x25,
    CMD_MAGNETIC_ENCODER = 0x26,
    CMD_MOTOR_VOLTAGE = 0x2A,
    CMD_GIMBAL_SYSTEM_INFO = 0x31,
    CMD_SET_LASER_RANGING = 0x32,
    CMD_GET_THERMAL_GAIN = 0x37,
    CMD_SET_THERMAL_GAIN = 0x38,
    CMD_SEND_GPS_DATA = 0x3E,
    CMD_FORMAT_SD = 0x48,
    CMD_TF_CARD_INFO = 0x49,
    CMD_GET_AI_MODE = 0x4D,
    CMD_GET_AI_STREAM_STATUS = 0x4E,
    CMD_UPDATE_THERMAL_SHUTTER = 0x4F,
    CMD_AI_TRACKING_DATA = 0x50,
    CMD_SET_AI_STREAM = 0x51,
    CMD_SET_AI_MODE = 0x55,
    CMD_AI_SELECT_TARGET = 0x56,
    CMD_AI_TRACKING_STATUS = 0x57,
    CMD_GET_EIS_STATUS = 0x60,
    CMD_SET_EIS_STATUS = 0x61,
    CMD_GET_IP_ADDRESS = 0x81,
    CMD_SET_IP_ADDRESS = 0x82
};

// Packet structure
#pragma pack(push, 1)
struct PacketHeader {
    uint16_t stx;           // Start flag 0x6655
    uint8_t ctrl;           // Control byte
    uint16_t data_len;      // Data length
    uint16_t seq;           // Sequence number
    uint8_t cmd_id;         // Command ID
};
#pragma pack(pop)

class UnipodProtocol {
public:
    UnipodProtocol();
    ~UnipodProtocol();

    // CRC16 calculation
    static uint16_t calculateCRC16(const uint8_t* data, uint32_t length);
    
    // Packet creation
    std::vector<uint8_t> createPacket(uint8_t cmd_id, const std::vector<uint8_t>& data, 
                                      bool need_ack = true);
    
    // Packet parsing
    bool parsePacket(const std::vector<uint8_t>& buffer, PacketHeader& header, 
                     std::vector<uint8_t>& data);
    
    // Verify CRC
    bool verifyCRC(const std::vector<uint8_t>& packet);
    
    // Get/Set sequence number
    uint16_t getSequenceNumber() { return seq_num_; }
    void setSequenceNumber(uint16_t seq) { seq_num_ = seq; }

    // Create specific command packets
    std::vector<uint8_t> createHeartbeatPacket();
    std::vector<uint8_t> createRequestFirmwarePacket();
    std::vector<uint8_t> createGimbalRotationPacket(int8_t yaw_speed, int8_t pitch_speed);
    std::vector<uint8_t> createSetGimbalAnglePacket(int16_t yaw, int16_t pitch);
    std::vector<uint8_t> createZoomPacket(int8_t zoom_cmd);
    std::vector<uint8_t> createAbsoluteZoomPacket(uint8_t zoom_int, uint8_t zoom_float);
    std::vector<uint8_t> createPhotoVideoPacket(uint8_t func_type);
    std::vector<uint8_t> createRequestGimbalAttitudePacket();
    std::vector<uint8_t> createSetLaserPacket(bool enable);
    std::vector<uint8_t> createRequestDataStreamPacket(uint8_t data_type, uint8_t freq);

private:
    uint16_t seq_num_;
    static const uint16_t crc16_tab_[256];
};

} // namespace Unipod_sdk

#endif // UNIPOD_PROTOCOL_HPP
