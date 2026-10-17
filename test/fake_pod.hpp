// In-memory pod for the client tests: every written frame goes to a responder, whose reply
// frames come back through read(). Can also push unsolicited frames and drop the link.
#pragma once

#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "unipod_driver/protocol.hpp"
#include "unipod_driver/transport.hpp"

namespace unipod_driver
{
namespace test
{

struct PodState
{
  std::mutex mtx;
  std::condition_variable cv;
  std::deque<uint8_t> rx;                    // bytes for the client to read
  std::vector<protocol::Frame> written;      // frames the client sent
  bool open{false};
  bool refuse_open{false};
  bool drop{false};                          // next read reports a lost link
  int opens{0};
  // cmd + payload in, reply payload out (nullopt: stay silent).
  std::function<std::optional<std::vector<uint8_t>>(const protocol::Frame &)> responder;

  void push(uint8_t cmd, const std::vector<uint8_t> & payload)
  {
    const auto f = protocol::encode(cmd, payload, 0, false);
    {
      std::lock_guard<std::mutex> lk(mtx);
      rx.insert(rx.end(), f.begin(), f.end());
    }
    cv.notify_all();
  }

  size_t count(uint8_t cmd)
  {
    std::lock_guard<std::mutex> lk(mtx);
    size_t n = 0;
    for (const auto & f : written) {
      n += f.cmd == cmd ? 1 : 0;
    }
    return n;
  }

  std::optional<protocol::Frame> last(uint8_t cmd)
  {
    std::lock_guard<std::mutex> lk(mtx);
    for (auto it = written.rbegin(); it != written.rend(); ++it) {
      if (it->cmd == cmd) {
        return *it;
      }
    }
    return std::nullopt;
  }
};

class FakePodTransport : public Transport
{
public:
  explicit FakePodTransport(std::shared_ptr<PodState> s)
  : s_(std::move(s)) {}

  bool open() override
  {
    std::lock_guard<std::mutex> lk(s_->mtx);
    if (s_->refuse_open) {
      return false;
    }
    s_->open = true;
    s_->drop = false;
    ++s_->opens;
    return true;
  }

  void close() override
  {
    std::lock_guard<std::mutex> lk(s_->mtx);
    s_->open = false;
  }

  bool isOpen() const override
  {
    std::lock_guard<std::mutex> lk(s_->mtx);
    return s_->open;
  }

  bool write(const std::vector<uint8_t> & data) override
  {
    std::vector<protocol::Frame> frames;
    parser_.push(data.data(), data.size(), [&](const protocol::Frame & f) {frames.push_back(f);});
    for (const auto & f : frames) {
      std::optional<std::vector<uint8_t>> reply;
      {
        std::lock_guard<std::mutex> lk(s_->mtx);
        if (!s_->open) {
          return false;
        }
        s_->written.push_back(f);
      }
      if (s_->responder) {
        reply = s_->responder(f);
      }
      if (reply) {
        s_->push(f.cmd, *reply);
      }
    }
    return true;
  }

  int read(uint8_t * buf, size_t len, int timeout_ms) override
  {
    std::unique_lock<std::mutex> lk(s_->mtx);
    s_->cv.wait_for(
      lk, std::chrono::milliseconds(timeout_ms), [&] {return !s_->rx.empty() || s_->drop;});
    if (s_->drop) {
      s_->drop = false;
      return -1;
    }
    size_t n = 0;
    while (n < len && !s_->rx.empty()) {
      buf[n++] = s_->rx.front();
      s_->rx.pop_front();
    }
    return static_cast<int>(n);
  }

  bool needsHeartbeat() const override {return true;}
  std::string describe() const override {return "fake://pod";}

private:
  std::shared_ptr<PodState> s_;
  protocol::FrameParser parser_;
};

}  // namespace test
}  // namespace unipod_driver
