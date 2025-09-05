#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lds01_driver/parser.h"
#include "lds01_driver/serial.h"
#include "lds01_driver/gpio.h"
#include <thread>
#include <atomic>
#include <poll.h>
#include <cstring>

extern "C" {
    #include "lds01_driver/parser.h"
    #include "lds01_driver/serial.h"
    #include "lds01_driver/gpio.h"
}

class LidarNode : public rclcpp::Node {
public:
    LidarNode() : Node("lidar_node"), running_(true) {
        // Параметры
        this->declare_parameter("serial_port", "/dev/ttyAMA0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("frame_id", "laser_frame");
        this->declare_parameter("inverted", true);
        this->declare_parameter("angle_compensate", true);
        
        // Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", 
            rclcpp::SensorDataQoS()
        );
        
        // Инициализация лидара
        init_lidar();

        // Запуск отдельного потока для чтения UART
        uart_thread_ = std::thread(&LidarNode::uart_read_loop, this);
        
        // Таймер для публикации данных
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&LidarNode::publish_scan, this));
            
        RCLCPP_INFO(this->get_logger(), "Lidar node started");
    }
    
    ~LidarNode() {
        running_ = false;
        if (uart_thread_.joinable()) {
            uart_thread_.join();
        }
        stop_motor();
        gpio_cleanup();
        if (uart_fd_ >= 0) {
            close_serial(uart_fd_);
        }
        RCLCPP_INFO(this->get_logger(), "Lidar node stopped");
    }

private:
    void init_lidar() {
        // Получаем параметры
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        inverted_ = this->get_parameter("inverted").as_bool();
        angle_compensate_ = this->get_parameter("angle_compensate").as_bool();
        
        // Инициализация GPIO
        if (gpio_init() < 0) {
            RCLCPP_ERROR(this->get_logger(), "GPIO initialization failed");
            //rclcpp::shutdown();
            return;
        }
        
        // Запуск мотора
        start_motor(1.0);
        RCLCPP_INFO(this->get_logger(), "Motor started");
        
        // Открытие UART
        uart_fd_ = open_serial(serial_port.c_str(), baud_rate);
        if (uart_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port.c_str());
            stop_motor();
	    gpio_cleanup();
	    //rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Serial port opened: %s", serial_port.c_str());
        
        // Инициализация парсера
        lidar_parser_init(&parser_);
    }

    void uart_read_loop() {
        struct pollfd fds[1];
        fds[0].fd = uart_fd_;
        fds[0].events = POLLIN;
        
        uint8_t buffer[256];
        
        while (running_) {
            int ready = poll(fds, 1, 100);
            if (!running_) break;
            
            if (ready == -1) {
                RCLCPP_ERROR(this->get_logger(), "Poll error: %s", strerror(errno));
                continue;
            }
            
            if (fds[0].revents & POLLIN) {
                int bytes_read = read_serial(uart_fd_, buffer, sizeof(buffer));
                if (bytes_read > 0) {
                    feed_data(&parser_, buffer, bytes_read);
                    
                    // Копируем данные скана когда они готовы
                    if (parser_.scan_ready) {
                        std::lock_guard<std::mutex> lock(scan_mutex_);
                        std::memcpy(scan_data_, parser_.scan, sizeof(scan_data_));
                        current_rpm_ = parser_.current_rpm;
                        scan_ready_ = true;
                    }
                } else if (bytes_read < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Read error: %s", strerror(errno));
                }
            }
        }
    }

    void publish_scan() {
        if (scan_ready_) {
            auto message = sensor_msgs::msg::LaserScan();
            
            // Заполняем заголовок
            message.header.stamp = this->now();
            message.header.frame_id = frame_id_;
            
            // Заполняем параметры скана
            message.angle_min = 0.0;
            message.angle_max = 2.0 * M_PI;
            message.angle_increment = 2.0 * M_PI / 360.0;
            
            // Проверка деления на ноль
            if (current_rpm_ > 0.0f) {
                message.time_increment = (1.0f / (current_rpm_ / 60.0f)) / 360.0f;
                message.scan_time = 1.0f / (current_rpm_ / 60.0f);
            } else {
                message.time_increment = 0.0;
                message.scan_time = 0.0;
            }
            
            message.range_min = 0.15f;
            message.range_max = 12.0f;
            
            // Заполняем данные дальностей
            message.ranges.resize(360);
            
            std::lock_guard<std::mutex> lock(scan_mutex_);
            if (inverted_) {
                for (int i = 0; i < 360; i++) {
                    message.ranges[i] = scan_data_[359 - i];
                }
            } else {
                for (int i = 0; i < 360; i++) {
                    message.ranges[i] = scan_data_[i];
                }
            }
            
            // Публикуем сообщение
            publisher_->publish(message);
            RCLCPP_DEBUG(this->get_logger(), "Scan published: rpm=%.2f", current_rpm_);
            
            scan_ready_ = false;
        }
    }

    // Члены класса
    int uart_fd_ = -1;
    lidar_parser_t parser_;
    std::thread uart_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> scan_ready_{false};
    float scan_data_[360];
    float current_rpm_ = 0.0f;
    std::mutex scan_mutex_;
    
    std::string frame_id_;
    bool inverted_;
    bool angle_compensate_;
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
