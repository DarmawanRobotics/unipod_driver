#include "unipod_driver/transport.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>

namespace unipod_driver
{

namespace
{

// Waits for `fd` to become readable; > 0 ready, 0 timeout, < 0 error/hangup.
int waitReadable(int fd, int timeout_ms)
{
  pollfd p{fd, POLLIN, 0};
  const int r = ::poll(&p, 1, timeout_ms);
  if (r < 0) {
    return errno == EINTR ? 0 : -1;
  }
  if (r == 0) {
    return 0;
  }
  if ((p.revents & POLLIN) != 0) {
    return 1;
  }
  return -1;  // POLLERR / POLLHUP / POLLNVAL
}

bool makeAddr(const std::string & ip, uint16_t port, sockaddr_in & addr)
{
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  return ::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) == 1;
}

class FdTransport : public Transport
{
public:
  ~FdTransport() override {FdTransport::close();}

  void close() override
  {
    const int fd = fd_.exchange(-1);
    if (fd >= 0) {
      ::close(fd);
    }
  }

  bool isOpen() const override {return fd_.load() >= 0;}

protected:
  std::atomic<int> fd_{-1};
};

class UdpTransport : public FdTransport
{
public:
  UdpTransport(std::string ip, uint16_t port)
  : ip_(std::move(ip)), port_(port) {}

  bool open() override
  {
    close();
    sockaddr_in addr{};
    if (!makeAddr(ip_, port_, addr)) {
      return false;
    }
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
      return false;
    }
    // connect() fixes the peer: send() goes to the pod, recv() only takes its datagrams.
    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      ::close(fd);
      return false;
    }
    fd_ = fd;
    return true;
  }

  bool write(const std::vector<uint8_t> & data) override
  {
    const int fd = fd_.load();
    return fd >= 0 && ::send(fd, data.data(), data.size(), 0) ==
           static_cast<ssize_t>(data.size());
  }

  int read(uint8_t * buf, size_t len, int timeout_ms) override
  {
    const int fd = fd_.load();
    if (fd < 0) {
      return -1;
    }
    const int w = waitReadable(fd, timeout_ms);
    if (w <= 0) {
      // ICMP port unreachable shows up as an error on a connected UDP socket: not fatal.
      return 0;
    }
    const ssize_t n = ::recv(fd, buf, len, 0);
    return n > 0 ? static_cast<int>(n) : 0;
  }

  std::string describe() const override {return "udp://" + ip_ + ":" + std::to_string(port_);}

private:
  std::string ip_;
  uint16_t port_;
};

class TcpTransport : public FdTransport
{
public:
  TcpTransport(std::string ip, uint16_t port)
  : ip_(std::move(ip)), port_(port) {}

  bool open() override
  {
    close();
    sockaddr_in addr{};
    if (!makeAddr(ip_, port_, addr)) {
      return false;
    }
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      return false;
    }
    // Non-blocking connect with a 1 s limit, so a missing pod never hangs the caller.
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    int r = ::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    if (r < 0 && errno == EINPROGRESS) {
      pollfd p{fd, POLLOUT, 0};
      r = ::poll(&p, 1, 1000) == 1 ? 0 : -1;
      int err = 0;
      socklen_t err_len = sizeof(err);
      if (r == 0 && (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &err_len) < 0 || err != 0)) {
        r = -1;
      }
    }
    if (r < 0) {
      ::close(fd);
      return false;
    }
    ::fcntl(fd, F_SETFL, flags);
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    fd_ = fd;
    return true;
  }

  bool write(const std::vector<uint8_t> & data) override
  {
    const int fd = fd_.load();
    return fd >= 0 && ::send(fd, data.data(), data.size(), MSG_NOSIGNAL) ==
           static_cast<ssize_t>(data.size());
  }

  int read(uint8_t * buf, size_t len, int timeout_ms) override
  {
    const int fd = fd_.load();
    if (fd < 0) {
      return -1;
    }
    const int w = waitReadable(fd, timeout_ms);
    if (w <= 0) {
      return w;
    }
    const ssize_t n = ::recv(fd, buf, len, 0);
    if (n == 0) {
      return -1;  // peer closed
    }
    if (n < 0) {
      return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
    }
    return static_cast<int>(n);
  }

  bool needsHeartbeat() const override {return true;}
  std::string describe() const override {return "tcp://" + ip_ + ":" + std::to_string(port_);}

private:
  std::string ip_;
  uint16_t port_;
};

class SerialTransport : public FdTransport
{
public:
  SerialTransport(std::string device, uint32_t baudrate)
  : device_(std::move(device)), baudrate_(baudrate) {}

  bool open() override
  {
    close();
    speed_t speed;
    switch (baudrate_) {
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      case 230400: speed = B230400; break;
      case 460800: speed = B460800; break;
      case 921600: speed = B921600; break;
      default: return false;
    }
    const int fd = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      return false;
    }
    termios tty{};
    if (::tcgetattr(fd, &tty) != 0) {
      ::close(fd);
      return false;
    }
    ::cfmakeraw(&tty);
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |= CS8 | CREAD | CLOCAL;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
      ::close(fd);
      return false;
    }
    ::tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    return true;
  }

  bool write(const std::vector<uint8_t> & data) override
  {
    const int fd = fd_.load();
    if (fd < 0) {
      return false;
    }
    size_t done = 0;
    while (done < data.size()) {
      const ssize_t n = ::write(fd, data.data() + done, data.size() - done);
      if (n < 0) {
        if (errno == EAGAIN || errno == EINTR) {
          pollfd p{fd, POLLOUT, 0};
          ::poll(&p, 1, 50);
          continue;
        }
        return false;
      }
      done += static_cast<size_t>(n);
    }
    return true;
  }

  int read(uint8_t * buf, size_t len, int timeout_ms) override
  {
    const int fd = fd_.load();
    if (fd < 0) {
      return -1;
    }
    const int w = waitReadable(fd, timeout_ms);
    if (w <= 0) {
      return w;  // unplugged adapter: POLLHUP / POLLERR -> reopen
    }
    const ssize_t n = ::read(fd, buf, len);
    if (n < 0) {
      return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
    }
    return static_cast<int>(n);
  }

  std::string describe() const override
  {
    return "serial://" + device_ + "@" + std::to_string(baudrate_);
  }

private:
  std::string device_;
  uint32_t baudrate_;
};

}  // namespace

std::unique_ptr<Transport> makeUdpTransport(const std::string & ip, uint16_t port)
{
  return std::make_unique<UdpTransport>(ip, port);
}

std::unique_ptr<Transport> makeTcpTransport(const std::string & ip, uint16_t port)
{
  return std::make_unique<TcpTransport>(ip, port);
}

std::unique_ptr<Transport> makeSerialTransport(const std::string & device, uint32_t baudrate)
{
  return std::make_unique<SerialTransport>(device, baudrate);
}

std::unique_ptr<Transport> makeTransport(
  const std::string & type, const std::string & ip, uint16_t port,
  const std::string & serial_device, uint32_t baudrate)
{
  if (type == "udp") {
    return makeUdpTransport(ip, port);
  }
  if (type == "tcp") {
    return makeTcpTransport(ip, port);
  }
  if (type == "serial") {
    return makeSerialTransport(serial_device, baudrate);
  }
  return nullptr;
}

}  // namespace unipod_driver
