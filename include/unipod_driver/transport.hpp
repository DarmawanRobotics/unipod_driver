// Byte transports to the pod: UDP / TCP on 37260 (192.168.144.25 by default), or TTL serial
// at 115200 8N1 (document chapter 3).
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace unipod_driver
{

class Transport
{
public:
  virtual ~Transport() = default;

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;
  virtual bool write(const std::vector<uint8_t> & data) = 0;
  // > 0 bytes read, 0 on timeout, < 0 when the link is gone (reopen it).
  virtual int read(uint8_t * buf, size_t len, int timeout_ms) = 0;
  // TCP needs the 0x00 heartbeat, the others do not.
  virtual bool needsHeartbeat() const {return false;}
  virtual std::string describe() const = 0;
};

std::unique_ptr<Transport> makeUdpTransport(const std::string & ip, uint16_t port);
std::unique_ptr<Transport> makeTcpTransport(const std::string & ip, uint16_t port);
std::unique_ptr<Transport> makeSerialTransport(const std::string & device, uint32_t baudrate);

// "udp" | "tcp" | "serial"; nullptr for anything else.
std::unique_ptr<Transport> makeTransport(
  const std::string & type, const std::string & ip, uint16_t port,
  const std::string & serial_device, uint32_t baudrate);

}  // namespace unipod_driver
