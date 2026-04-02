#ifndef UNIPOD_COMM_HPP
#define UNIOPOD_COMM_HPP

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

namespace unipod_driver {

enum ConnectionType {
    CONN_TCP,
    CONN_UDP,
    CONN_SERIAL
};

class UnipodComm {
public:
    using ReceiveCallback = std::function<void(const std::vector<uint8_t>&)>;

    UnipodComm();
    ~UnipodComm();

    // Connection methods
    bool connectTCP(const std::string& ip, uint16_t port);
    bool connectUDP(const std::string& ip, uint16_t port);
    bool connectSerial(const std::string& device, uint32_t baudrate);
    void disconnect();
    
    bool isConnected() const { return connected_; }
    ConnectionType getConnectionType() const { return conn_type_; }

    // Send/Receive
    bool sendData(const std::vector<uint8_t>& data);
    void setReceiveCallback(ReceiveCallback callback) { receive_callback_ = callback; }

    // Heartbeat for TCP
    void startHeartbeat(uint32_t interval_ms = 1000);
    void stopHeartbeat();

private:
    std::vector<uint8_t> rx_buffer_;
    std::mutex rx_mutex_;
    void receiveThread();
    void heartbeatThread();
    bool setupSerialPort(const std::string& device, uint32_t baudrate);

    ConnectionType conn_type_;
    int socket_fd_;
    int serial_fd_;
    std::string ip_address_;
    uint16_t port_;
    struct sockaddr_in server_addr_;
    
    std::atomic<bool> connected_;
    std::atomic<bool> running_;
    std::thread receive_thread_;
    std::thread heartbeat_thread_;
    
    ReceiveCallback receive_callback_;
    std::mutex send_mutex_;
    
    uint32_t heartbeat_interval_ms_;
    std::atomic<bool> heartbeat_running_;
};

} // namespace unipod_driver

#endif // UNIPOD_COMM_HPP
