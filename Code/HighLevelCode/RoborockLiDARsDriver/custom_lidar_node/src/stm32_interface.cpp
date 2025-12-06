#include "stm32_interface.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

STM32Interface::STM32Interface(rclcpp::Node* node) 
    : node_(node) {
}

STM32Interface::~STM32Interface() {
    disconnect();
}

bool STM32Interface::connect(const std::string& port, int baudrate) {
    port_ = port;
    
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open STM32 port %s", port_.c_str());
        return false;
    }
    
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get serial attributes");
        close(fd_);
        fd_ = -1;
        return false;
    }
    
    // Настройки порта
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);
    
    tty.c_cflag &= ~PARENB;     // No parity
    tty.c_cflag &= ~CSTOPB;     // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         // 8 bits
    tty.c_cflag &= ~CRTSCTS;    // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable reading
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    
    tty.c_cc[VMIN]  = 0;    // Non-blocking
    tty.c_cc[VTIME] = 10;   // 1 second timeout
    
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set serial attributes");
        close(fd_);
        fd_ = -1;
        return false;
    }
    
    tcflush(fd_, TCIOFLUSH);
    
    RCLCPP_INFO(node_->get_logger(), "Connected to STM32 on %s", port_.c_str());
    return true;
}

void STM32Interface::disconnect() {
    if (fd_ >= 0) {
        sendEmergencyStop();
        close(fd_);
        fd_ = -1;
        RCLCPP_INFO(node_->get_logger(), "Disconnected from STM32");
    }
}

std::vector<uint8_t> STM32Interface::encodeCommand(const MotorCommand& cmd) {
    std::vector<uint8_t> packet(13);
    
    // Заголовок
    packet[0] = 0xAA;  // Start byte 1
    packet[1] = 0x55;  // Start byte 2
    packet[2] = 0x08;  // Длина данных
    
    // Команда: движение (0x01)
    packet[3] = 0x01;
    
    // Данные: vx, vy, omega (float, little-endian)
    float vx = std::clamp(cmd.vx, -2.0f, 2.0f);
    float vy = std::clamp(cmd.vy, -2.0f, 2.0f);
    float omega = std::clamp(cmd.omega, -3.14f, 3.14f);
    
    uint8_t* vx_bytes = reinterpret_cast<uint8_t*>(&vx);
    uint8_t* vy_bytes = reinterpret_cast<uint8_t*>(&vy);
    uint8_t* omega_bytes = reinterpret_cast<uint8_t*>(&omega);
    
    for (int i = 0; i < 4; i++) {
        packet[4 + i] = vx_bytes[i];
        packet[8 + i] = vy_bytes[i];
        packet[12 + i] = omega_bytes[i];
    }
    
    // Контрольная сумма (XOR всех байт, кроме заголовка)
    uint8_t checksum = 0;
    for (size_t i = 2; i < packet.size(); i++) {
        checksum ^= packet[i];
    }
    packet.push_back(checksum);
    
    return packet;
}

bool STM32Interface::sendPacket(const std::vector<uint8_t>& data) {
    if (fd_ < 0) {
        return false;
    }
    
    ssize_t written = write(fd_, data.data(), data.size());
    if (written != static_cast<ssize_t>(data.size())) {
        RCLCPP_WARN(node_->get_logger(), "Failed to send complete packet to STM32");
        return false;
    }
    
    tcdrain(fd_); // Ждем отправки всех данных
    return true;
}

bool STM32Interface::sendCommand(const MotorCommand& cmd) {
    auto packet = encodeCommand(cmd);
    return sendPacket(packet);
}

bool STM32Interface::sendEmergencyStop() {
    std::vector<uint8_t> packet = {0xAA, 0x55, 0x01, 0x02, 0x02}; // Команда аварийной остановки
    return sendPacket(packet);
}