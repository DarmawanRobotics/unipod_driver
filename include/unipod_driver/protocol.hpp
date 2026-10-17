// UniPod MT11 external SDK protocol (Reebot, V0.1.0): framing, CRC16, little-endian helpers.
// No ROS here, so it can be unit-tested against the document's own example packets.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace unipod_driver
{
namespace protocol
{

constexpr uint8_t kStx0 = 0x55;  // STX 0x6655, little endian on the wire
constexpr uint8_t kStx1 = 0x66;
constexpr size_t kHeaderSize = 8;
constexpr size_t kCrcSize = 2;
constexpr size_t kMinFrameSize = kHeaderSize + kCrcSize;
constexpr size_t kMaxPayload = 1024;

constexpr uint8_t kCtrlNeedAck = 0x01;
constexpr uint8_t kCtrlAckPack = 0x02;

// Only the commands this driver uses (PTZ, zoom, video mode, thermal, reset, info).
namespace cmd
{
constexpr uint8_t kHeartbeat = 0x00;
constexpr uint8_t kFirmwareVersion = 0x01;
constexpr uint8_t kManualZoom = 0x05;
constexpr uint8_t kRotate = 0x07;
constexpr uint8_t kCenter = 0x08;
constexpr uint8_t kAttitude = 0x0D;
constexpr uint8_t kSetAngle = 0x0E;
constexpr uint8_t kAbsoluteZoom = 0x0F;
constexpr uint8_t kGetVideoMode = 0x10;
constexpr uint8_t kSetVideoMode = 0x11;
constexpr uint8_t kTempPoint = 0x12;
constexpr uint8_t kTempRegion = 0x13;
constexpr uint8_t kTempFrame = 0x14;
constexpr uint8_t kZoomRange = 0x16;
constexpr uint8_t kCurrentZoom = 0x18;
constexpr uint8_t kGetPalette = 0x1A;
constexpr uint8_t kSetPalette = 0x1B;
constexpr uint8_t kGetEncoding = 0x20;
constexpr uint8_t kDataStream = 0x25;
constexpr uint8_t kGetThermalGain = 0x37;
constexpr uint8_t kSetThermalGain = 0x38;
constexpr uint8_t kThermalShutter = 0x4F;
}  // namespace cmd

struct Frame
{
  uint8_t ctrl{0};
  uint16_t seq{0};
  uint8_t cmd{0};
  std::vector<uint8_t> payload;
};

// CRC16-CCITT (X^16+X^12+X^5+1), table driven, as in chapter 5 of the document.
uint16_t crc16(const uint8_t * data, size_t len, uint16_t init = 0);

// Full frame: STX, CTRL, len, SEQ, CMD, payload, CRC.
std::vector<uint8_t> encode(
  uint8_t cmd, const std::vector<uint8_t> & payload, uint16_t seq, bool need_ack = true);

// Byte-stream reassembly: resyncs on STX, drops frames with a bad CRC or absurd length.
class FrameParser
{
public:
  using Handler = std::function<void (const Frame &)>;

  void push(const uint8_t * data, size_t len, const Handler & on_frame);
  void reset() {buf_.clear();}
  size_t crcErrors() const {return crc_errors_;}

private:
  std::vector<uint8_t> buf_;
  size_t crc_errors_{0};
};

// Little-endian payload writer.
class Writer
{
public:
  Writer & u8(uint8_t v) {b_.push_back(v); return *this;}
  Writer & i8(int8_t v) {return u8(static_cast<uint8_t>(v));}
  Writer & u16(uint16_t v)
  {
    b_.push_back(static_cast<uint8_t>(v & 0xFF));
    b_.push_back(static_cast<uint8_t>(v >> 8));
    return *this;
  }
  Writer & i16(int16_t v) {return u16(static_cast<uint16_t>(v));}
  const std::vector<uint8_t> & bytes() const {return b_;}

private:
  std::vector<uint8_t> b_;
};

inline uint16_t readU16(const std::vector<uint8_t> & b, size_t off)
{
  return static_cast<uint16_t>(b[off] | (b[off + 1] << 8));
}
inline int16_t readI16(const std::vector<uint8_t> & b, size_t off)
{
  return static_cast<int16_t>(readU16(b, off));
}
inline uint32_t readU32(const std::vector<uint8_t> & b, size_t off)
{
  return static_cast<uint32_t>(b[off]) | (static_cast<uint32_t>(b[off + 1]) << 8) |
         (static_cast<uint32_t>(b[off + 2]) << 16) | (static_cast<uint32_t>(b[off + 3]) << 24);
}

}  // namespace protocol
}  // namespace unipod_driver
