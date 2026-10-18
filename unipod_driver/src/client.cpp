#include "unipod_driver/client.hpp"

#include <algorithm>
#include <cmath>

namespace unipod_driver
{

using protocol::readI16;
using protocol::readU16;
using protocol::readU32;
using protocol::Writer;
namespace cmd = protocol::cmd;
using Clock = std::chrono::steady_clock;

UnipodClient::UnipodClient(std::unique_ptr<Transport> transport, Options options)
: transport_(std::move(transport)), options_(options) {}

UnipodClient::~UnipodClient() {stop();}

void UnipodClient::start()
{
  if (running_.exchange(true)) {
    return;
  }
  rx_thread_ = std::thread(&UnipodClient::rxLoop, this);
}

void UnipodClient::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  wake_cv_.notify_all();
  pending_cv_.notify_all();
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  std::lock_guard<std::mutex> lk(write_mtx_);
  transport_->close();
  link_up_ = false;
}

void UnipodClient::log(const std::string & msg) const
{
  if (on_log_) {
    on_log_(msg);
  }
}

bool UnipodClient::sleepFor(std::chrono::milliseconds d)
{
  std::unique_lock<std::mutex> lk(wake_mtx_);
  wake_cv_.wait_for(lk, d, [this] {return !running_.load();});
  return running_.load();
}

bool UnipodClient::send(uint8_t command, const std::vector<uint8_t> & payload, bool need_ack)
{
  const auto frame = protocol::encode(command, payload, seq_.fetch_add(1), need_ack);
  std::lock_guard<std::mutex> lk(write_mtx_);
  return transport_->isOpen() && transport_->write(frame);
}

std::optional<std::vector<uint8_t>> UnipodClient::request(
  uint8_t command, const std::vector<uint8_t> & payload, size_t min_reply_len)
{
  std::lock_guard<std::mutex> one_in_flight(cmd_mtx_[command]);
  for (int attempt = 0; attempt <= options_.retries && running_; ++attempt) {
    auto pending = std::make_shared<Pending>();
    {
      std::lock_guard<std::mutex> lk(pending_mtx_);
      pending_[command] = pending;
    }
    if (!send(command, payload, true)) {
      std::lock_guard<std::mutex> lk(pending_mtx_);
      pending_.erase(command);
      return std::nullopt;  // no transport: fail fast instead of waiting out the timeout
    }
    std::unique_lock<std::mutex> lk(pending_mtx_);
    pending_cv_.wait_for(
      lk, options_.timeout, [&] {return pending->done || !running_.load();});
    pending_.erase(command);
    if (pending->done) {
      if (pending->payload.size() < min_reply_len) {
        return std::nullopt;
      }
      return pending->payload;
    }
  }
  return std::nullopt;
}

// ---------------------------------------------------------------- receive side

void UnipodClient::rxLoop()
{
  protocol::FrameParser parser;
  std::array<uint8_t, 2048> buf{};
  auto last_heartbeat = Clock::time_point{};
  auto last_probe = Clock::time_point{};
  last_rx_ = Clock::now();

  while (running_) {
    if (!transport_->isOpen()) {
      bool opened;
      {
        std::lock_guard<std::mutex> lk(write_mtx_);
        opened = transport_->open();
      }
      if (!opened) {
        sleepFor(options_.reconnect_period);
        continue;
      }
      parser.reset();
      last_rx_ = Clock::now();
      log("opened " + transport_->describe());
    }

    const auto now = Clock::now();
    if (transport_->needsHeartbeat() && now - last_heartbeat >= options_.heartbeat_period) {
      send(cmd::kHeartbeat, {0x00}, true);
      last_heartbeat = now;
    }
    // The heartbeat is not answered: probe with a firmware request while nothing arrives, so
    // the link state is known and a UDP peer is woken up.
    if (now - last_rx_ >= std::chrono::milliseconds(1000) &&
      now - last_probe >= std::chrono::milliseconds(1000))
    {
      send(cmd::kFirmwareVersion);
      last_probe = now;
    }

    const int n = transport_->read(buf.data(), buf.size(), 100);
    if (n < 0) {
      log("link lost on " + transport_->describe() + ", reconnecting");
      {
        std::lock_guard<std::mutex> lk(write_mtx_);
        transport_->close();
      }
      link_up_ = false;
      sleepFor(options_.reconnect_period);
      continue;
    }
    if (n > 0) {
      parser.push(buf.data(), static_cast<size_t>(n),
        [this](const protocol::Frame & f) {handleFrame(f);});
    }
    if (link_up_ && Clock::now() - last_rx_ > options_.link_timeout) {
      link_up_ = false;
      log("no reply from " + transport_->describe());
    }
  }
}

void UnipodClient::handleFrame(const protocol::Frame & frame)
{
  last_rx_ = Clock::now();
  if (!link_up_.exchange(true)) {
    log("link up on " + transport_->describe());
    resendStreams();
  }

  {
    std::lock_guard<std::mutex> lk(pending_mtx_);
    auto it = pending_.find(frame.cmd);
    if (it != pending_.end() && !it->second->done) {
      it->second->done = true;
      it->second->payload = frame.payload;
      pending_cv_.notify_all();
    }
  }

  // Pushed data (streams), also produced by the replies to the matching requests.
  if (frame.cmd == cmd::kAttitude && frame.payload.size() >= 12 && on_attitude_) {
    on_attitude_(decodeAttitude(frame.payload));
  } else if (frame.cmd == cmd::kTempFrame && frame.payload.size() >= 12 && on_frame_temp_) {
    on_frame_temp_(decodeFrame(frame.payload));
  }
}

void UnipodClient::resendStreams()
{
  std::map<uint8_t, uint8_t> streams;
  bool frame_temp;
  {
    std::lock_guard<std::mutex> lk(streams_mtx_);
    streams = streams_;
    frame_temp = frame_temp_stream_;
  }
  for (const auto & [type, rate] : streams) {
    send(cmd::kDataStream, Writer().u8(type).u8(rate).bytes());
  }
  if (frame_temp) {
    send(cmd::kTempFrame, Writer().u8(2).bytes());
  }
}

// ---------------------------------------------------------------- decoders

Attitude UnipodClient::decodeAttitude(const std::vector<uint8_t> & p)
{
  Attitude a;
  a.yaw = readI16(p, 0) / 10.0f;
  a.pitch = readI16(p, 2) / 10.0f;
  a.roll = readI16(p, 4) / 10.0f;
  if (p.size() >= 12) {
    a.yaw_rate = readI16(p, 6) / 10.0f;
    a.pitch_rate = readI16(p, 8) / 10.0f;
    a.roll_rate = readI16(p, 10) / 10.0f;
  }
  return a;
}

Temperature UnipodClient::decodePoint(const std::vector<uint8_t> & p)
{
  Temperature t;
  t.mode = Temperature::kPoint;
  t.max_c = t.min_c = readU16(p, 0) / 100.0f;
  t.x1 = t.x2 = t.max_x = t.min_x = readU16(p, 2);
  t.y1 = t.y2 = t.max_y = t.min_y = readU16(p, 4);
  return t;
}

Temperature UnipodClient::decodeRegion(const std::vector<uint8_t> & p)
{
  Temperature t;
  t.mode = Temperature::kRegion;
  t.x1 = readU16(p, 0);
  t.y1 = readU16(p, 2);
  t.x2 = readU16(p, 4);
  t.y2 = readU16(p, 6);
  t.max_c = readU16(p, 8) / 100.0f;
  t.min_c = readU16(p, 10) / 100.0f;
  t.max_x = readU16(p, 12);
  t.max_y = readU16(p, 14);
  t.min_x = readU16(p, 16);
  t.min_y = readU16(p, 18);
  return t;
}

Temperature UnipodClient::decodeFrame(const std::vector<uint8_t> & p)
{
  Temperature t;
  t.mode = Temperature::kFrame;
  t.max_c = readU16(p, 0) / 100.0f;
  t.min_c = readU16(p, 2) / 100.0f;
  t.max_x = readU16(p, 4);
  t.max_y = readU16(p, 6);
  t.min_x = readU16(p, 8);
  t.min_y = readU16(p, 10);
  return t;
}

std::string UnipodClient::versionString(uint32_t v)
{
  // Top byte is the product code (0x8A gimbal, 0x89 camera): 0x8A00010A -> v0.1.10.
  return "v" + std::to_string((v >> 16) & 0xFF) + "." + std::to_string((v >> 8) & 0xFF) + "." +
         std::to_string(v & 0xFF);
}

FirmwareInfo UnipodClient::decodeFirmware(const std::vector<uint8_t> & p)
{
  FirmwareInfo f;
  f.camera = versionString(readU32(p, 0));
  f.gimbal = versionString(readU32(p, 4));
  return f;
}

float UnipodClient::decodeZoom(const std::vector<uint8_t> & p)
{
  return static_cast<float>(p[0]) + static_cast<float>(p[1]) / 10.0f;
}

// ---------------------------------------------------------------- PTZ

bool UnipodClient::rotate(int8_t yaw_speed, int8_t pitch_speed)
{
  const auto clamp = [](int8_t v) {
      return static_cast<int8_t>(std::clamp<int>(v, -100, 100));
    };
  return send(cmd::kRotate, Writer().i8(clamp(yaw_speed)).i8(clamp(pitch_speed)).bytes());
}

std::optional<Attitude> UnipodClient::setAngle(float yaw_deg, float pitch_deg)
{
  pitch_deg = std::clamp(pitch_deg, -90.0f, 30.0f);
  const auto yaw = static_cast<int16_t>(std::lround(yaw_deg * 10.0f));
  const auto pitch = static_cast<int16_t>(std::lround(pitch_deg * 10.0f));
  auto r = request(cmd::kSetAngle, Writer().i16(yaw).i16(pitch).bytes(), 6);
  if (!r) {
    return std::nullopt;
  }
  return decodeAttitude(*r);
}

bool UnipodClient::recenter(CenterMode mode)
{
  auto r = request(cmd::kCenter, Writer().u8(mode).bytes(), 1);
  return r && (*r)[0] == 1;
}

std::optional<Attitude> UnipodClient::getAttitude()
{
  auto r = request(cmd::kAttitude, {}, 12);
  if (!r) {
    return std::nullopt;
  }
  return decodeAttitude(*r);
}

bool UnipodClient::setAttitudeStream(StreamRate rate)
{
  {
    std::lock_guard<std::mutex> lk(streams_mtx_);
    if (rate == kRateOff) {
      streams_.erase(kDataAttitude);
    } else {
      streams_[kDataAttitude] = rate;
    }
  }
  auto r = request(cmd::kDataStream, Writer().u8(kDataAttitude).u8(rate).bytes(), 1);
  return r.has_value();
}

// ---------------------------------------------------------------- zoom

std::optional<float> UnipodClient::zoom(int8_t direction)
{
  direction = static_cast<int8_t>(std::clamp<int>(direction, -1, 1));
  auto r = request(cmd::kManualZoom, Writer().i8(direction).bytes(), 2);
  if (!r) {
    return std::nullopt;
  }
  return readU16(*r, 0) / 10.0f;
}

bool UnipodClient::setZoom(float level)
{
  const long tenths = std::lround(std::clamp(level, 1.0f, 30.0f) * 10.0f);
  auto r = request(
    cmd::kAbsoluteZoom,
    Writer().u8(static_cast<uint8_t>(tenths / 10)).u8(static_cast<uint8_t>(tenths % 10)).bytes(),
    1);
  return r && (*r)[0] == 1;
}

std::optional<float> UnipodClient::getZoom()
{
  auto r = request(cmd::kCurrentZoom, {}, 2);
  if (!r) {
    return std::nullopt;
  }
  return decodeZoom(*r);
}

std::optional<float> UnipodClient::getMaxZoom()
{
  auto r = request(cmd::kZoomRange, {}, 2);
  if (!r) {
    return std::nullopt;
  }
  return decodeZoom(*r);
}

// ---------------------------------------------------------------- video

std::optional<VideoMode> UnipodClient::getVideoMode()
{
  auto r = request(cmd::kGetVideoMode, {}, 2);
  if (!r) {
    return std::nullopt;
  }
  return VideoMode{(*r)[0], (*r)[1]};
}

std::optional<VideoMode> UnipodClient::setVideoMode(uint8_t main_stream, uint8_t sub_stream)
{
  auto r = request(cmd::kSetVideoMode, Writer().u8(main_stream).u8(sub_stream).bytes(), 2);
  if (!r) {
    return std::nullopt;
  }
  return VideoMode{(*r)[0], (*r)[1]};
}

std::optional<Encoding> UnipodClient::getEncoding(uint8_t stream)
{
  auto r = request(cmd::kGetEncoding, Writer().u8(stream).bytes(), 9);
  if (!r) {
    return std::nullopt;
  }
  Encoding e;
  e.stream = (*r)[0];
  e.codec = (*r)[1];
  e.width = readU16(*r, 2);
  e.height = readU16(*r, 4);
  e.bitrate_kbps = readU16(*r, 6);
  e.fps = (*r)[8];
  return e;
}

// ---------------------------------------------------------------- thermal

std::optional<Temperature> UnipodClient::pointTemperature(uint16_t x, uint16_t y)
{
  auto r = request(cmd::kTempPoint, Writer().u16(x).u16(y).u8(1).bytes(), 6);
  if (!r) {
    return std::nullopt;
  }
  return decodePoint(*r);
}

std::optional<Temperature> UnipodClient::regionTemperature(
  uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  auto r = request(
    cmd::kTempRegion, Writer().u16(x1).u16(y1).u16(x2).u16(y2).u8(1).bytes(), 20);
  if (!r) {
    return std::nullopt;
  }
  return decodeRegion(*r);
}

std::optional<Temperature> UnipodClient::frameTemperature()
{
  auto r = request(cmd::kTempFrame, Writer().u8(1).bytes(), 12);
  if (!r) {
    return std::nullopt;
  }
  return decodeFrame(*r);
}

bool UnipodClient::setFrameTemperatureStream(bool enable)
{
  {
    std::lock_guard<std::mutex> lk(streams_mtx_);
    frame_temp_stream_ = enable;
  }
  // flag 2: continuous at 5 Hz, 0: off. The stream frames themselves are the answer.
  return send(cmd::kTempFrame, Writer().u8(enable ? 2 : 0).bytes());
}

std::optional<uint8_t> UnipodClient::getPalette()
{
  auto r = request(cmd::kGetPalette, {}, 1);
  if (!r) {
    return std::nullopt;
  }
  return (*r)[0];
}

std::optional<uint8_t> UnipodClient::setPalette(uint8_t palette)
{
  auto r = request(cmd::kSetPalette, Writer().u8(palette).bytes(), 1);
  if (!r) {
    return std::nullopt;
  }
  return (*r)[0];
}

std::optional<uint8_t> UnipodClient::getThermalGain()
{
  auto r = request(cmd::kGetThermalGain, {}, 1);
  if (!r) {
    return std::nullopt;
  }
  return (*r)[0];
}

std::optional<uint8_t> UnipodClient::setThermalGain(uint8_t gain)
{
  auto r = request(cmd::kSetThermalGain, Writer().u8(gain ? 1 : 0).bytes(), 1);
  if (!r) {
    return std::nullopt;
  }
  return (*r)[0];
}

bool UnipodClient::refreshShutter()
{
  auto r = request(cmd::kThermalShutter, {}, 1);
  return r && (*r)[0] == 1;
}

std::optional<FirmwareInfo> UnipodClient::getFirmware()
{
  auto r = request(cmd::kFirmwareVersion, {}, 8);
  if (!r) {
    return std::nullopt;
  }
  return decodeFirmware(*r);
}

}  // namespace unipod_driver
