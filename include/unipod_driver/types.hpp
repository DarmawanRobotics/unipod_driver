// Decoded UniPod MT11 replies, in SI-ish units (degrees, degrees Celsius, zoom factor).
#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace unipod_driver
{

// Video streams (0x10 / 0x11).
enum Stream : uint8_t
{
  kStreamZoom = 0,
  kStreamWide = 1,
  kStreamThermal = 2,
  kStreamZoomThermal = 3,  // zoom stitched with thermal
  kStreamWideThermal = 4,
  kStreamZoomWide = 5,
  kStreamNone = 6,
};

// Gimbal recenter (0x08).
enum CenterMode : uint8_t
{
  kCenterReset = 1,       // one-key reset
  kCenterAndDown = 2,     // center, then face downward
  kCenterOnly = 3,
  kCenterFaceDown = 4,
};

// 0x25 data stream rates.
enum StreamRate : uint8_t
{
  kRateOff = 0, kRate2Hz = 1, kRate4Hz = 2, kRate5Hz = 3,
  kRate10Hz = 4, kRate20Hz = 5, kRate50Hz = 6, kRate100Hz = 7,
};

// The supported 0x25 rate closest to `hz` (<= 0: off).
inline StreamRate nearestStreamRate(double hz)
{
  if (!(hz > 0.0)) {
    return kRateOff;
  }
  constexpr double kHz[] = {2, 4, 5, 10, 20, 50, 100};
  int best = 0;
  for (int i = 1; i < 7; ++i) {
    if (std::abs(kHz[i] - hz) < std::abs(kHz[best] - hz)) {
      best = i;
    }
  }
  return static_cast<StreamRate>(best + 1);
}

inline double streamRateHz(StreamRate r)
{
  constexpr double kHz[] = {0, 2, 4, 5, 10, 20, 50, 100};
  return r <= kRate100Hz ? kHz[r] : 0.0;
}

// -1..1 stick value to a 0x07 speed (-max..max, max <= 100); NaN/inf give 0.
inline int8_t axisToSpeed(double axis, int max_speed)
{
  if (!std::isfinite(axis)) {
    return 0;
  }
  axis = axis < -1.0 ? -1.0 : (axis > 1.0 ? 1.0 : axis);
  max_speed = max_speed < 0 ? 0 : (max_speed > 100 ? 100 : max_speed);
  return static_cast<int8_t>(std::lround(axis * max_speed));
}

// -1..1 stick value to a 0x05 zoom direction, with a dead band of +-0.5.
inline int8_t axisToZoom(double axis)
{
  if (!std::isfinite(axis)) {
    return 0;
  }
  return axis > 0.5 ? 1 : (axis < -0.5 ? -1 : 0);
}

// 0x25 data types.
enum DataStream : uint8_t
{
  kDataAttitude = 1,
  kDataLaser = 2,
  kDataEncoder = 3,
  kDataMotorVoltage = 4,
};

// Gimbal attitude, NED, rotation order yaw -> roll -> pitch (0x0D / 0x0E).
// Yaw positive to the right, pitch positive up; degrees and degrees per second.
struct Attitude
{
  float yaw{0.f};
  float pitch{0.f};
  float roll{0.f};
  float yaw_rate{0.f};
  float pitch_rate{0.f};
  float roll_rate{0.f};
};

// Temperature measurement (0x12 point, 0x13 region, 0x14 full frame), degrees Celsius,
// pixel coordinates of the thermal stream.
struct Temperature
{
  enum Mode : uint8_t { kPoint = 0, kRegion = 1, kFrame = 2 };
  Mode mode{kFrame};
  uint16_t x1{0}, y1{0}, x2{0}, y2{0};  // point: x1/y1; region: the box the camera used
  float max_c{0.f};
  float min_c{0.f};
  uint16_t max_x{0}, max_y{0};
  uint16_t min_x{0}, min_y{0};
};

struct VideoMode
{
  uint8_t main{kStreamZoom};
  uint8_t sub{kStreamThermal};
};

// Camera encoding (0x20).
struct Encoding
{
  uint8_t stream{1};   // 0 record, 1 main, 2 sub
  uint8_t codec{1};    // 1 H264, 2 H265
  uint16_t width{0};
  uint16_t height{0};
  uint16_t bitrate_kbps{0};
  uint8_t fps{0};
};

struct FirmwareInfo
{
  std::string camera;  // "v1.0.1", v0.0.0 during the first ~30 s after power-up
  std::string gimbal;
};

// Thermal palettes (0x1A / 0x1B).
enum Palette : uint8_t
{
  kWhiteHot = 0, kSepia = 2, kIronbow = 3, kRainbow = 4, kNight = 5, kAurora = 6,
  kRedHot = 7, kJungle = 8, kMedical = 9, kBlackHot = 10, kGloryHot = 11,
};

}  // namespace unipod_driver
