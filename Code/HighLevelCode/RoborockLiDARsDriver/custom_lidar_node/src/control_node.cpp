#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "movement.h"
#include "stm32_interface.h"
#include <memory>
#include <mutex>

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        // Параметры
        this->declare_parameter<std::string>("stm32_port", "/tmp/stm32_mock");
        this->declare_parameter<int>("stm32_baud", 115200);
        
        // Конфигурация алгоритма
        LidarConfig config;
        config.min_range = this->declare_parameter<float>("min_range", 0.2f);
        config.max_range = this->declare_parameter<float>("max_range", 2.5f);
        config.safety_margin = this->declare_parameter<float>("safety_margin", 0.3f);
        config.base_speed = this->declare_parameter<float>("base_speed", 0.3f);
        config.body_block_start = this->declare_parameter<float>("body_block_start", 150.0f);
        config.body_block_end = this->declare_parameter<float>("body_block_end", 210.0f);
        
        // Инициализация
        movement_ = std::make_unique<Movement>(config);
        stm32_interface_ = std::make_unique<STM32Interface>(this);
        
        // Подключение к STM32 (или моку)
        std::string stm32_port = this->get_parameter("stm32_port").as_string();
        int stm32_baud = this->get_parameter("stm32_baud").as_int();
        
        if (!stm32_interface_->connect(stm32_port, stm32_baud)) {
            RCLCPP_WARN(this->get_logger(), "Failed to connect to STM32, using mock mode");
        }
        
        // Подписка на сканы
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&ControlNode::scanCallback, this, std::placeholders::_1));
        
        // Таймер управления
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControlNode::controlLoop, this));
        
        // Публикация команд
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Control node started");
    }
    
    ~ControlNode() {
        if (stm32_interface_) {
            stm32_interface_->disconnect();
        }
    }
    
private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        last_scan_ = *msg;
        scan_received_ = true;
    }
    
    void controlLoop() {
        sensor_msgs::msg::LaserScan current_scan;
        bool has_scan = false;
        
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            if (scan_received_) {
                current_scan = last_scan_;
                has_scan = true;
            }
        }
        
        if (!has_scan) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "No scan data received");
            return;
        }
        
        // Обработка скана
        std::vector<float> ranges(current_scan.ranges.begin(), current_scan.ranges.end());
        MotorCommand cmd = movement_->processScan(ranges);
        
        // Отправка команды на STM32
        if (stm32_interface_->isConnected()) {
            if (!stm32_interface_->sendCommand(cmd)) {
                RCLCPP_WARN(this->get_logger(), "Failed to send command to STM32");
            }
        }
        
        // Публикация для визуализации
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = cmd.vx;
        twist_msg.linear.y = cmd.vy;
        twist_msg.angular.z = cmd.omega;
        cmd_pub_->publish(twist_msg);
        
        // Логирование
        static int log_counter = 0;
        if (log_counter++ % 25 == 0) {
            std::string state_str;
            if (movement_->isEmergencyStop()) {
                state_str = "EMERGENCY_STOP";
            } else if (movement_->isAvoiding()) {
                state_str = "AVOIDING";
            } else {
                state_str = "FOLLOWING";
            }
            
            RCLCPP_INFO(this->get_logger(), 
                       "State: %s, Cmd: vx=%.2f, vy=%.2f, omega=%.2f",
                       state_str.c_str(), cmd.vx, cmd.vy, cmd.omega);
        }
    }
    
    std::unique_ptr<STM32Interface> stm32_interface_;
    std::unique_ptr<Movement> movement_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    std::mutex scan_mutex_;
    sensor_msgs::msg::LaserScan last_scan_;
    bool scan_received_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}