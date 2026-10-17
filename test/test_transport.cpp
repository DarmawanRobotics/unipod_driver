// Real UDP and TCP sockets on the loopback interface, with a minimal pod on the other end.
#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <thread>

#include "unipod_driver/client.hpp"

using namespace std::chrono_literals;
using unipod_driver::UnipodClient;
using unipod_driver::protocol::encode;
using unipod_driver::protocol::Frame;
using unipod_driver::protocol::FrameParser;

namespace
{

// Answers 0x18 (current zoom) with 4.2x, counts heartbeats.
class LoopbackPod
{
public:
  explicit LoopbackPod(int type)
  : type_(type)
  {
    fd_ = ::socket(AF_INET, type, 0);
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    ::bind(fd_, reinterpret_cast<sockaddr *>(&a), sizeof(a));
    socklen_t len = sizeof(a);
    ::getsockname(fd_, reinterpret_cast<sockaddr *>(&a), &len);
    port_ = ntohs(a.sin_port);
    if (type == SOCK_STREAM) {
      ::listen(fd_, 1);
    }
    thread_ = std::thread([this] {run();});
  }

  ~LoopbackPod()
  {
    running_ = false;
    thread_.join();
    ::close(fd_);
  }

  uint16_t port() const {return port_;}
  int heartbeats() const {return heartbeats_;}

private:
  void handle(const Frame & f, int fd, const sockaddr_in * peer)
  {
    if (f.cmd == 0x00) {
      ++heartbeats_;
      return;
    }
    if (f.cmd != 0x18) {
      return;
    }
    const auto reply = encode(0x18, {4, 2}, f.seq, false);
    if (peer) {
      ::sendto(fd, reply.data(), reply.size(), 0,
        reinterpret_cast<const sockaddr *>(peer), sizeof(*peer));
    } else {
      ::send(fd, reply.data(), reply.size(), MSG_NOSIGNAL);
    }
  }

  void run()
  {
    int conn = -1;
    FrameParser parser;
    uint8_t buf[512];
    while (running_) {
      int fd = fd_;
      if (type_ == SOCK_STREAM && conn < 0) {
        pollfd p{fd_, POLLIN, 0};
        if (::poll(&p, 1, 20) == 1) {
          conn = ::accept(fd_, nullptr, nullptr);
        }
        continue;
      }
      if (type_ == SOCK_STREAM) {
        fd = conn;
      }
      pollfd p{fd, POLLIN, 0};
      if (::poll(&p, 1, 20) != 1) {
        continue;
      }
      sockaddr_in peer{};
      socklen_t plen = sizeof(peer);
      const ssize_t n = type_ == SOCK_DGRAM ?
        ::recvfrom(fd, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&peer), &plen) :
        ::recv(fd, buf, sizeof(buf), 0);
      if (n <= 0) {
        continue;
      }
      parser.push(buf, static_cast<size_t>(n), [&](const Frame & f) {
          handle(f, fd, type_ == SOCK_DGRAM ? &peer : nullptr);
        });
    }
    if (conn >= 0) {
      ::close(conn);
    }
  }

  int type_;
  int fd_{-1};
  uint16_t port_{0};
  std::atomic<bool> running_{true};
  std::atomic<int> heartbeats_{0};
  std::thread thread_;
};

template<typename Pred>
bool waitFor(Pred pred, std::chrono::milliseconds limit = 3000ms)
{
  const auto end = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < end && !pred()) {
    std::this_thread::sleep_for(5ms);
  }
  return pred();
}

}  // namespace

TEST(Transport, UdpRequestReply)
{
  LoopbackPod pod(SOCK_DGRAM);
  UnipodClient client(unipod_driver::makeUdpTransport("127.0.0.1", pod.port()));
  client.start();
  std::optional<float> zoom;
  ASSERT_TRUE(waitFor([&] {zoom = client.getZoom(); return zoom.has_value();}));
  EXPECT_NEAR(*zoom, 4.2f, 1e-4);
  EXPECT_TRUE(client.linkUp());
  EXPECT_EQ(pod.heartbeats(), 0);  // UDP: no heartbeat
}

TEST(Transport, TcpRequestReplyAndHeartbeat)
{
  LoopbackPod pod(SOCK_STREAM);
  UnipodClient client(unipod_driver::makeTcpTransport("127.0.0.1", pod.port()));
  client.start();
  std::optional<float> zoom;
  ASSERT_TRUE(waitFor([&] {zoom = client.getZoom(); return zoom.has_value();}));
  EXPECT_NEAR(*zoom, 4.2f, 1e-4);
  EXPECT_TRUE(waitFor([&] {return pod.heartbeats() >= 1;}));
}

TEST(Transport, TcpConnectFailsQuicklyWhenNobodyListens)
{
  auto t = unipod_driver::makeTcpTransport("127.0.0.1", 1);  // nothing on port 1
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(t->open());
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1500ms);
}

TEST(Transport, FactoryRejectsUnknownType)
{
  EXPECT_EQ(unipod_driver::makeTransport("can", "1.2.3.4", 1, "", 115200), nullptr);
  EXPECT_NE(unipod_driver::makeTransport("udp", "1.2.3.4", 1, "", 115200), nullptr);
  EXPECT_FALSE(unipod_driver::makeSerialTransport("/dev/does-not-exist", 115200)->open());
  EXPECT_FALSE(unipod_driver::makeSerialTransport("/dev/null", 1234)->open());  // bad baud
}
