// UniPod MT11 client: one receive thread, request/ACK matching by command id, link
// supervision (heartbeat, probe, reconnect) and re-sending the active data streams after the
// link comes back. No ROS dependency.
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "unipod_driver/protocol.hpp"
#include "unipod_driver/transport.hpp"
#include "unipod_driver/types.hpp"

namespace unipod_driver
{

class UnipodClient
{
public:
  struct Options
  {
    std::chrono::milliseconds timeout{500};          // per request attempt
    int retries{1};                                  // extra attempts after a timeout
    std::chrono::milliseconds link_timeout{3000};    // no frame for this long: link down
    std::chrono::milliseconds heartbeat_period{1000};
    std::chrono::milliseconds reconnect_period{1000};
  };

  using AttitudeCallback = std::function<void (const Attitude &)>;
  using TemperatureCallback = std::function<void (const Temperature &)>;
  using LogCallback = std::function<void (const std::string &)>;

  explicit UnipodClient(std::unique_ptr<Transport> transport, Options options);
  explicit UnipodClient(std::unique_ptr<Transport> transport)
  : UnipodClient(std::move(transport), Options()) {}
  ~UnipodClient();

  UnipodClient(const UnipodClient &) = delete;
  UnipodClient & operator=(const UnipodClient &) = delete;

  // Callbacks run on the receive thread; set them before start().
  void onAttitude(AttitudeCallback cb) {on_attitude_ = std::move(cb);}
  void onFrameTemperature(TemperatureCallback cb) {on_frame_temp_ = std::move(cb);}
  void onLog(LogCallback cb) {on_log_ = std::move(cb);}

  void start();
  void stop();
  bool linkUp() const {return link_up_.load();}
  std::string describe() const {return transport_->describe();}

  // Raw access.
  bool send(uint8_t cmd, const std::vector<uint8_t> & payload = {}, bool need_ack = true);
  std::optional<std::vector<uint8_t>> request(
    uint8_t cmd, const std::vector<uint8_t> & payload, size_t min_reply_len);

  // PTZ. Speeds -100..100, yaw positive right, pitch positive up. Fire and forget.
  bool rotate(int8_t yaw_speed, int8_t pitch_speed);
  // Absolute angle, yaw unlimited, pitch -90..30 deg. Returns the attitude in the ACK
  // (angles only).
  std::optional<Attitude> setAngle(float yaw_deg, float pitch_deg);
  bool recenter(CenterMode mode);
  std::optional<Attitude> getAttitude();
  // Pushed attitude through onAttitude(); remembered and re-sent after a reconnect.
  bool setAttitudeStream(StreamRate rate);

  // Zoom (hybrid, auto focus). direction 1 in, 0 stop, -1 out; returns the zoom level.
  std::optional<float> zoom(int8_t direction);
  bool setZoom(float level);
  std::optional<float> getZoom();
  std::optional<float> getMaxZoom();

  // Video streams.
  std::optional<VideoMode> getVideoMode();
  std::optional<VideoMode> setVideoMode(uint8_t main_stream, uint8_t sub_stream);
  std::optional<Encoding> getEncoding(uint8_t stream);

  // Thermal: temperatures in degrees Celsius, coordinates in thermal stream pixels.
  std::optional<Temperature> pointTemperature(uint16_t x, uint16_t y);
  std::optional<Temperature> regionTemperature(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
  std::optional<Temperature> frameTemperature();
  // Full-frame max/min at 5 Hz through onFrameTemperature(); remembered like the attitude.
  bool setFrameTemperatureStream(bool enable);
  std::optional<uint8_t> getPalette();
  std::optional<uint8_t> setPalette(uint8_t palette);
  std::optional<uint8_t> getThermalGain();
  std::optional<uint8_t> setThermalGain(uint8_t gain);  // 0 low, 1 high
  bool refreshShutter();

  std::optional<FirmwareInfo> getFirmware();

  // Reply decoders, public for the tests.
  static Attitude decodeAttitude(const std::vector<uint8_t> & p);
  static Temperature decodePoint(const std::vector<uint8_t> & p);
  static Temperature decodeRegion(const std::vector<uint8_t> & p);
  static Temperature decodeFrame(const std::vector<uint8_t> & p);
  static FirmwareInfo decodeFirmware(const std::vector<uint8_t> & p);
  static std::string versionString(uint32_t v);
  static float decodeZoom(const std::vector<uint8_t> & p);  // int + tenth bytes

private:
  struct Pending
  {
    bool done{false};
    std::vector<uint8_t> payload;
  };

  void rxLoop();
  void handleFrame(const protocol::Frame & frame);
  void resendStreams();
  void log(const std::string & msg) const;
  bool sleepFor(std::chrono::milliseconds d);

  std::unique_ptr<Transport> transport_;
  Options options_;

  std::atomic<bool> running_{false};
  std::atomic<bool> link_up_{false};
  std::atomic<uint16_t> seq_{0};
  std::thread rx_thread_;
  std::mutex write_mtx_;
  std::mutex wake_mtx_;
  std::condition_variable wake_cv_;

  std::mutex pending_mtx_;
  std::condition_variable pending_cv_;
  std::map<uint8_t, std::shared_ptr<Pending>> pending_;
  std::array<std::mutex, 256> cmd_mtx_;

  std::mutex streams_mtx_;
  std::map<uint8_t, uint8_t> streams_;  // 0x25 data type -> rate
  bool frame_temp_stream_{false};

  std::chrono::steady_clock::time_point last_rx_{};

  AttitudeCallback on_attitude_;
  TemperatureCallback on_frame_temp_;
  LogCallback on_log_;
};

}  // namespace unipod_driver
