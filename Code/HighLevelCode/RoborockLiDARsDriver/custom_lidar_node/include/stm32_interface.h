#ifndef STM32_INTERFACE_H
#define STM32_INTERFACE_H

#include <rclcpp/rclcpp/rclcpp.hpp>
// #include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>

struct MotorCommand {
    float vx;          // линейная скорость по X (м/с)
    float vy;          // линейная скорость по Y (м/с)
    float omega;       // угловая скорость (рад/с)
    
    MotorCommand(float vx = 0, float vy = 0, float omega = 0) 
        : vx(vx), vy(vy), omega(omega) {}
};

class STM32Interface {
public:
    STM32Interface(rclcpp::Node* node);
    ~STM32Interface();
    
    bool connect(const std::string& port = "/dev/ttyACM0", int baudrate = 115200);
    void disconnect();
    
    bool sendCommand(const MotorCommand& cmd);
    bool sendEmergencyStop();
    
    bool isConnected() const { return fd_ >= 0; }
    
private:
    int fd_{-1};
    rclcpp::Node* node_;
    std::string port_;
    
    bool sendPacket(const std::vector<uint8_t>& data);
    std::vector<uint8_t> encodeCommand(const MotorCommand& cmd);
};

#endif