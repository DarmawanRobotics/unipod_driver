#include "unipod_driver/unipod_protocol.hpp"
#include <iostream>

namespace unipod_driver {

// CRC16 lookup table
const uint16_t UnipodProtocol::crc16_tab_[256] = {
    0x0, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0xa50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0xc60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0xe70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0xa1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x2b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x8e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0xaf1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0xcc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0xed1, 0x1ef0
};

UnipodProtocol::UnipodProtocol() : seq_num_(0) {}

UnipodProtocol::~UnipodProtocol() {}

uint16_t UnipodProtocol::calculateCRC16(const uint8_t* data, uint32_t length) {
    uint16_t crc = 0;
    
    for (uint32_t i = 0; i < length; i++) {
        uint8_t temp = (crc >> 8) & 0xFF;
        uint16_t oldcrc16 = crc16_tab_[data[i] ^ temp];
        crc = (crc << 8) ^ oldcrc16;
    }
    
    return crc;
}

std::vector<uint8_t> UnipodProtocol::createPacket(uint8_t cmd_id, 
                                                 const std::vector<uint8_t>& data,
                                                 bool need_ack) {
    std::vector<uint8_t> packet;
    
    // Calculate total packet size
    uint16_t data_len = data.size();
    size_t total_size = PROTOCOL_HEADER_SIZE + data_len + PROTOCOL_CRC_SIZE;
    packet.resize(total_size);
    
    // Fill header
    size_t offset = 0;
    
    // STX (Little Endian)
    packet[offset++] = PROTOCOL_STX & 0xFF;
    packet[offset++] = (PROTOCOL_STX >> 8) & 0xFF;
    
    // CTRL
    uint8_t ctrl = need_ack ? 0x01 : 0x00;
    packet[offset++] = ctrl;
    
    // Data Length (Little Endian)
    packet[offset++] = data_len & 0xFF;
    packet[offset++] = (data_len >> 8) & 0xFF;
    
    // SEQ (Little Endian)
    packet[offset++] = seq_num_ & 0xFF;
    packet[offset++] = (seq_num_ >> 8) & 0xFF;
    seq_num_++;
    
    // CMD_ID
    packet[offset++] = cmd_id;
    
    // Data
    if (data_len > 0) {
        std::memcpy(&packet[offset], data.data(), data_len);
        offset += data_len;
    }
    
    // CRC16 (Little Endian)
    uint16_t crc = calculateCRC16(packet.data(), offset);
    packet[offset++] = crc & 0xFF;
    packet[offset++] = (crc >> 8) & 0xFF;
    
    return packet;
}

bool UnipodProtocol::parsePacket(const std::vector<uint8_t>& buffer, 
                               PacketHeader& header, 
                               std::vector<uint8_t>& data) {
    if (buffer.size() < PROTOCOL_HEADER_SIZE + PROTOCOL_CRC_SIZE) {
        return false;
    }
    
    // Parse header
    size_t offset = 0;
    header.stx = buffer[offset] | (buffer[offset + 1] << 8);
    offset += 2;
    
    if (header.stx != PROTOCOL_STX) {
        return false;
    }
    
    header.ctrl = buffer[offset++];
    header.data_len = buffer[offset] | (buffer[offset + 1] << 8);
    offset += 2;
    header.seq = buffer[offset] | (buffer[offset + 1] << 8);
    offset += 2;
    header.cmd_id = buffer[offset++];
    
    // Check packet size
   if (buffer.size() < PROTOCOL_HEADER_SIZE + static_cast<size_t>(header.data_len) + PROTOCOL_CRC_SIZE) {
        return false;
    }
    
    // Extract data
    if (header.data_len > 0) {
        data.resize(header.data_len);
        std::memcpy(data.data(), &buffer[offset], header.data_len);
    }
    
    return true;
}

bool UnipodProtocol::verifyCRC(const std::vector<uint8_t>& packet) {
    if (packet.size() < PROTOCOL_HEADER_SIZE + PROTOCOL_CRC_SIZE) {
        return false;
    }
    
    size_t crc_offset = packet.size() - PROTOCOL_CRC_SIZE;
    uint16_t received_crc = packet[crc_offset] | (packet[crc_offset + 1] << 8);
    uint16_t calculated_crc = calculateCRC16(packet.data(), crc_offset);
    
    return received_crc == calculated_crc;
}

// Specific command packet creators
std::vector<uint8_t> UnipodProtocol::createHeartbeatPacket() {
    return createPacket(CMD_TCP_HEARTBEAT, {}, true);
}

std::vector<uint8_t> UnipodProtocol::createRequestFirmwarePacket() {
    return createPacket(CMD_FIRMWARE_VERSION, {}, true);
}

std::vector<uint8_t> UnipodProtocol::createGimbalRotationPacket(int8_t yaw_speed, 
                                                               int8_t pitch_speed) {
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(yaw_speed),
        static_cast<uint8_t>(pitch_speed)
    };
    return createPacket(CMD_GIMBAL_ROTATION, data, true);
}

std::vector<uint8_t> UnipodProtocol::createSetGimbalAnglePacket(int16_t yaw, int16_t pitch) {
    std::vector<uint8_t> data(4);
    data[0] = yaw & 0xFF;
    data[1] = (yaw >> 8) & 0xFF;
    data[2] = pitch & 0xFF;
    data[3] = (pitch >> 8) & 0xFF;
    return createPacket(CMD_SET_GIMBAL_ANGLE, data, true);
}

std::vector<uint8_t> UnipodProtocol::createZoomPacket(int8_t zoom_cmd) {
    std::vector<uint8_t> data = {static_cast<uint8_t>(zoom_cmd)};
    return createPacket(CMD_MANUAL_ZOOM, data, true);
}

std::vector<uint8_t> UnipodProtocol::createAbsoluteZoomPacket(uint8_t zoom_int, 
                                                             uint8_t zoom_float) {
    std::vector<uint8_t> data = {zoom_int, zoom_float};
    return createPacket(CMD_ABSOLUTE_ZOOM, data, true);
}

std::vector<uint8_t> UnipodProtocol::createPhotoVideoPacket(uint8_t func_type) {
    std::vector<uint8_t> data = {func_type};
    return createPacket(CMD_PHOTO_VIDEO, data, false);
}

std::vector<uint8_t> UnipodProtocol::createRequestGimbalAttitudePacket() {
    return createPacket(CMD_GIMBAL_ATTITUDE, {}, true);
}

std::vector<uint8_t> UnipodProtocol::createSetLaserPacket(bool enable) {
    std::vector<uint8_t> data = {static_cast<uint8_t>(enable ? 1 : 0)};
    return createPacket(CMD_SET_LASER_RANGING, data, true);
}

std::vector<uint8_t> UnipodProtocol::createRequestDataStreamPacket(uint8_t data_type, 
                                                                  uint8_t freq) {
    std::vector<uint8_t> data = {data_type, freq};
    return createPacket(CMD_REQUEST_DATA_STREAM, data, true);
}

} // namespace unipod_driver
