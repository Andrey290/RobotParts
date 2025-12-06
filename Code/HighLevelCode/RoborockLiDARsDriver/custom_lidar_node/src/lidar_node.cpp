// src/lidar_node.cpp
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <chrono>
#include <cstring>
#include <atomic>

extern "C" {
#include "../include/parser.h"
#include "../include/serial.h"
#include "../include/gpio.h"
}

using namespace std::chrono_literals;

struct ScanMsg {
  std::array<float,360> ranges;
  float rpm;
  uint64_t timestamp_ms;
};

class LidarNode : public rclcpp::Node {
public:
  LidarNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("lidar_node"), running_(true) {
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    // параметры
    this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
    this->declare_parameter<int>("baud", 115200);
    this->declare_parameter<int>("pwm_gpio", 18);
    this->declare_parameter<double>("motor_duty", 1.0);

    serial_port_ = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud").as_int();
    pwm_gpio_ = this->get_parameter("pwm_gpio").as_int();
    double motor_duty = this->get_parameter("motor_duty").as_double();

    // open serial
    speed_t speed = B115200;
    switch(baud) {
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      case 230400: speed = B230400; break;
      case 460800: speed = B460800; break;
      default: speed = B115200;
    }
    fd_ = open_serial(serial_port_.c_str(), speed);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "open_serial failed");
      // but continue — maybe we'll retry
    } else {
      RCLCPP_INFO(this->get_logger(), "Opened serial %s", serial_port_.c_str());
    }

    // init gpio (pigpio)
    if (gpio_init() < 0) {
      RCLCPP_ERROR(this->get_logger(), "gpio_init failed");
    } else {
      start_motor(motor_duty); // duty 0..1 in your gpio.c
      RCLCPP_INFO(this->get_logger(), "Motor started at duty %.2f", motor_duty);
    }

    // init parser with callback
    lidar_parser_init(&parser_);
    parser_.cb = &LidarNode::static_scan_ready_cb;
    parser_.cb_user = this;

    // start reader thread
    reader_thread_ = std::thread(&LidarNode::reader_loop, this);

    // publisher timer
    timer_ = this->create_wall_timer(20ms, std::bind(&LidarNode::publish_loop, this));
  }

  ~LidarNode() override {
    running_.store(false);
    if (reader_thread_.joinable()) reader_thread_.join();
    stop_motor();
    gpio_cleanup();
    if (fd_ >= 0) close_serial(fd_);
  }

private:
  // static callback adapter — called from parser.c when a scan completes
  static void static_scan_ready_cb(const float *scan360, float rpm, void *user) {
    LidarNode *self = static_cast<LidarNode*>(user);
    self->on_scan_ready(scan360, rpm);
  }

  void on_scan_ready(const float *scan360, float rpm) {
    // copy to queue (thread-safe)
    ScanMsg m;
    for (int i=0;i<360;i++) m.ranges[i] = scan360[i]; // scan360 in meters per parser
    m.rpm = rpm;
    m.timestamp_ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
    );

    std::unique_lock<std::mutex> lk(q_mtx_);
    if (scan_q_.size() >= 4) scan_q_.pop(); // drop oldest
    scan_q_.push(std::move(m));
    lk.unlock();
    cv_.notify_one();
  }

  void reader_loop() {
    const size_t bufsize = 1024;
    std::vector<uint8_t> buf(bufsize);
    while (running_.load()) {
      if (fd_ < 0) {
        std::this_thread::sleep_for(200ms);
        continue;
      }
      ssize_t r = read(fd_, buf.data(), bufsize);
      if (r > 0) {
        feed_data(&parser_, buf.data(), (size_t)r);
      } else {
        // no data
        std::this_thread::sleep_for(2ms);
      }
    }
  }

  void publish_loop() {
    // if queue empty -> nothing
    std::unique_lock<std::mutex> lk(q_mtx_);
    if (scan_q_.empty()) return;
    ScanMsg m = std::move(scan_q_.front());
    scan_q_.pop();
    lk.unlock();

    auto msg = sensor_msgs::msg::LaserScan();
    // header
    msg.header.stamp = this->now();
    msg.header.frame_id = "laser_link";
    msg.angle_min = 0.0f;
    msg.angle_max = 2.0f * M_PI * (359.0f/360.0f);
    msg.angle_increment = (2.0f * M_PI) / 360.0f;
    if (m.rpm > 0.001f) {
      msg.scan_time = 60.0f / m.rpm;
      msg.time_increment = msg.scan_time / 360.0f;
    } else {
      msg.scan_time = 0.0f;
      msg.time_increment = 0.0f;
    }
    msg.ranges.resize(360);
    for (int i=0;i<360;i++) {
      float v = m.ranges[i];
      if (std::isnan(v) || v <= 0.0f) msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      else msg.ranges[i] = v;
    }
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published scan valid=%zu rpm=%.2f", count_valid(msg.ranges), m.rpm);
  }

  size_t count_valid(const std::vector<float>& v) {
    size_t c=0;
    for (auto &x: v) if (!std::isnan(x)) ++c;
    return c;
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string serial_port_;
  int fd_{-1};
  int pwm_gpio_{18};
  lidar_parser_t parser_;
  std::atomic<bool> running_;
  std::thread reader_thread_;
  std::mutex q_mtx_;
  std::condition_variable cv_;
  std::queue<ScanMsg> scan_q_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

