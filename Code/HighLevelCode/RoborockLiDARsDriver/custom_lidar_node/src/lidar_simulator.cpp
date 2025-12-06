#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <random>
#include <chrono>

class LidarSimulator : public rclcpp::Node {
public:
    LidarSimulator() : Node("lidar_simulator") {
        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&LidarSimulator::publishScan, this));
        
        RCLCPP_INFO(this->get_logger(), "LiDAR Simulator started");
    }

private:
    void publishScan() {
        auto msg = sensor_msgs::msg::LaserScan();
        msg.header.stamp = this->now();
        msg.header.frame_id = "laser_link";
        msg.angle_min = 0.0f;
        msg.angle_max = 2.0f * M_PI * (359.0f/360.0f);
        msg.angle_increment = (2.0f * M_PI) / 360.0f;
        msg.scan_time = 0.05f;
        msg.time_increment = msg.scan_time / 360.0f;
        msg.range_min = 0.2f;
        msg.range_max = 2.5f;
        
        msg.ranges.resize(360);
        
        std::default_random_engine generator;
        std::normal_distribution<float> distribution(1.5f, 0.3f);
        
        // Создаем "стену" впереди
        for (int i = 0; i < 360; i++) {
            float angle = i * 1.0f;
            
            // Сектор с препятствием (30 градусов спереди)
            if ((angle < 15 || angle > 345) && obstacle_active_) {
                msg.ranges[i] = 0.8f + 0.1f * sin(obstacle_counter_ * 0.1f);
            }
            // Случайный шум
            else if (rand() % 100 > 5) {
                msg.ranges[i] = distribution(generator);
            }
            // Некоторые точки как "дыры"
            else {
                msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            }
        }
        
        // Переключаем препятствие каждые 5 секунд
        obstacle_counter_++;
        if (obstacle_counter_ % 100 == 0) {
            obstacle_active_ = !obstacle_active_;
            RCLCPP_INFO(this->get_logger(), "Obstacle %s", 
                       obstacle_active_ ? "ACTIVE" : "INACTIVE");
        }
        
        pub_->publish(msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool obstacle_active_{false};
    int obstacle_counter_{0};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}