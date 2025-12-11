#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <random>
#include "fake_sensor_driver/fake_sensor_node.hpp"
#include <chrono>
#include <cmath> // Required for M_PI and sin function

using namespace std::chrono_literals;

FakeSensorNode::FakeSensorNode() : Node("fake_sensor_node")
{
  // 声明参数
  declare_parameter<double>("frequency", 10.0);  // 默认10Hz
  declare_parameter<std::string>("mode", "random");  // 模式：random 或 sine
  declare_parameter<double>("amplitude", 10.0);  // 正弦波振幅
  declare_parameter<double>("offset", 0.0);  // 偏移量
  declare_parameter<double>("noise", 0.1);  // 噪声水平

  // 获取参数
  double frequency = get_parameter("frequency").as_double();
  mode_ = get_parameter("mode").as_string();
  amplitude_ = get_parameter("amplitude").as_double();
  offset_ = get_parameter("offset").as_double();
  noise_ = get_parameter("noise").as_double();

  // 创建发布者
  sensor_publisher_ = this->create_publisher<std_msgs::msg::Float64>("sensor_data", 10);

  // 创建定时器
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / frequency),
    std::bind(&FakeSensorNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Fake Sensor Node started");
  RCLCPP_INFO(this->get_logger(), "Frequency: %.2f Hz", frequency);
  RCLCPP_INFO(this->get_logger(), "Mode: %s", mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "Amplitude: %.2f", amplitude_);
  RCLCPP_INFO(this->get_logger(), "Offset: %.2f", offset_);
  RCLCPP_INFO(this->get_logger(), "Noise: %.2f", noise_);
}

void FakeSensorNode::timer_callback()
{
  auto message = std_msgs::msg::Float64();
  
  if (mode_ == "random") {
    // 生成随机数
    message.data = offset_ + distribution_(generator_) * amplitude_ + distribution_(generator_) * noise_;
  } else if (mode_ == "sine") {
    // 生成正弦波
    double time = this->now().seconds();
    message.data = offset_ + amplitude_ * sin(2 * M_PI * 2 * time) + distribution_(generator_) * noise_;
  }

  sensor_publisher_->publish(message);
  RCLCPP_DEBUG(this->get_logger(), "Published data: %.4f", message.data);
  
  // 更新计数器
  ++count_;
}