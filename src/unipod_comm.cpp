#include "unipod_driver/unipod_comm.hpp"
#include "unipod_driver/unipod_protocol.hpp"
#include <iostream>
#include <cstring>
#include <chrono>

namespace unipod_driver {

UnipodComm::UnipodComm() 
    : conn_type_(CONN_TCP),
      socket_fd_(-1),
      serial_fd_(-1),
      port_(0),
      connected_(false),
      running_(false),
      heartbeat_interval_ms_(1000),
      heartbeat_running_(false) {
}

UnipodComm::~UnipodComm() {
    disconnect();
}

bool UnipodComm::connectTCP(const std::string& ip, uint16_t port) {
    if (connected_) {
        std::cerr << "Already connected" << std::endl;
        return false;
    }
    
    conn_type_ = CONN_TCP;
    ip_address_ = ip;
    port_ = port;
    
    // Create socket
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create TCP socket" << std::endl;
        return false;
    }
    
    // Setup server address
    std::memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port);
    
    if (inet_pton(AF_INET, ip.c_str(), &server_addr_.sin_addr) <= 0) {
        std::cerr << "Invalid address: " << ip << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    // Connect
    if (connect(socket_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
        std::cerr << "TCP connection failed" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    // Set non-blocking
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    
    connected_ = true;
    running_ = true;
    
    // Start receive thread
    receive_thread_ = std::thread(&UnipodComm::receiveThread, this);
    
    std::cout << "TCP connected to " << ip << ":" << port << std::endl;
    return true;
}

bool UnipodComm::connectUDP(const std::string& ip, uint16_t port) {
    if (connected_) {
        std::cerr << "Already connected" << std::endl;
        return false;
    }
    
    conn_type_ = CONN_UDP;
    ip_address_ = ip;
    port_ = port;
    
    // Create socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create UDP socket" << std::endl;
        return false;
    }
    
    // Setup server address
    std::memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port);
    
    if (inet_pton(AF_INET, ip.c_str(), &server_addr_.sin_addr) <= 0) {
        std::cerr << "Invalid address: " << ip << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    // Set non-blocking
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    
    connected_ = true;
    running_ = true;
    
    // Start receive thread
    receive_thread_ = std::thread(&UnipodComm::receiveThread, this);
    
    std::cout << "UDP connected to " << ip << ":" << port << std::endl;
    return true;
}

bool UnipodComm::connectSerial(const std::string& device, uint32_t baudrate) {
    if (connected_) {
        std::cerr << "Already connected" << std::endl;
        return false;
    }
    
    conn_type_ = CONN_SERIAL;
    
    if (!setupSerialPort(device, baudrate)) {
        return false;
    }
    
    connected_ = true;
    running_ = true;
    
    // Start receive thread
    receive_thread_ = std::thread(&UnipodComm::receiveThread, this);
    
    std::cout << "Serial connected to " << device << " at " << baudrate << " baud" << std::endl;
    return true;
}

void UnipodComm::disconnect() {
    if (!connected_) {
        return;
    }
    
    connected_ = false;
    running_ = false;
    
    stopHeartbeat();
    
    // Wait for threads to finish
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    
    // Close sockets/serial
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
    
    std::cout << "Disconnected" << std::endl;
}

bool UnipodComm::sendData(const std::vector<uint8_t>& data) {
    if (!connected_) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(send_mutex_);
    
    ssize_t sent = 0;
    
    switch (conn_type_) {
        case CONN_TCP:
            sent = send(socket_fd_, data.data(), data.size(), 0);
            break;
            
        case CONN_UDP:
            sent = sendto(socket_fd_, data.data(), data.size(), 0,
                         (struct sockaddr*)&server_addr_, sizeof(server_addr_));
            break;
            
        case CONN_SERIAL:
            sent = write(serial_fd_, data.data(), data.size());
            break;
    }
    
    return sent == static_cast<ssize_t>(data.size());
}

void UnipodComm::receiveThread() {
    std::vector<uint8_t> buffer(4096);

    while (running_) {
        ssize_t received = 0;

        switch (conn_type_) {
            case CONN_TCP:
                received = recv(socket_fd_, buffer.data(), buffer.size(), 0);
                break;

            case CONN_UDP: {
                socklen_t len = sizeof(server_addr_);
                received = recvfrom(socket_fd_, buffer.data(), buffer.size(), 0,
                                    (struct sockaddr*)&server_addr_, &len);
                break;
            }

            case CONN_SERIAL:
                received = read(serial_fd_, buffer.data(), buffer.size());
                break;
        }

        if (received > 0) {
            std::lock_guard<std::mutex> lock(rx_mutex_);

            // append ke buffer global
            rx_buffer_.insert(rx_buffer_.end(), buffer.begin(), buffer.begin() + received);

            // parsing loop
            while (rx_buffer_.size() >= 10) { // minimal size

                if (!(rx_buffer_[0] == 0x55 && rx_buffer_[1] == 0x66)) {
                    rx_buffer_.erase(rx_buffer_.begin());
                    continue;
                }

                uint16_t data_len = rx_buffer_[3] | (rx_buffer_[4] << 8);

                size_t full_size = 8 + data_len + 2; 

                if (rx_buffer_.size() < full_size) {
                    break;
                }

                std::vector<uint8_t> packet(
                    rx_buffer_.begin(),
                    rx_buffer_.begin() + full_size
                );

                if (receive_callback_) {
                    receive_callback_(packet);
                }

                rx_buffer_.erase(
                    rx_buffer_.begin(),
                    rx_buffer_.begin() + full_size
                );
            }

        } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            if (conn_type_ == CONN_TCP) {
                std::cerr << "TCP receive error, disconnecting" << std::endl;
                connected_ = false;
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void UnipodComm::startHeartbeat(uint32_t interval_ms) {
    if (conn_type_ != CONN_TCP) {
        std::cerr << "Heartbeat only supported for TCP connections" << std::endl;
        return;
    }
    
    heartbeat_interval_ms_ = interval_ms;
    heartbeat_running_ = true;
    heartbeat_thread_ = std::thread(&UnipodComm::heartbeatThread, this);
}

void UnipodComm::stopHeartbeat() {
    heartbeat_running_ = false;
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
}

void UnipodComm::heartbeatThread() {
    UnipodProtocol protocol;
    
    while (heartbeat_running_ && connected_) {
        auto heartbeat_packet = protocol.createHeartbeatPacket();
        sendData(heartbeat_packet);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_interval_ms_));
    }
}

bool UnipodComm::setupSerialPort(const std::string& device, uint32_t baudrate) {
    serial_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    if (serial_fd_ < 0) {
        std::cerr << "Failed to open serial port: " << device << std::endl;
        return false;
    }
    
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    // Set baudrate
    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        default:
            std::cerr << "Unsupported baudrate: " << baudrate << std::endl;
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // 8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr" << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    return true;
}

} // namespace unipod_driver
