#include "rclcpp/rclcpp.hpp"
#include "fake_capture_msgs/msg/captured_data.hpp"  // 使用带有时间戳的消息类型
#include <random>
#include "fake_sensor_driver/fake_sensor_node.hpp"
#include <chrono>
#include <cmath>  // 添加缺失的头文件

using namespace std::chrono_literals;

namespace fake_sensor_driver {

FakeSensorNode::FakeSensorNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("fake_sensor_node", options)  // 调用父类构造函数并提供节点名称和选项
{
  // 声明参数
  // 将默认频率从10Hz改为20Hz
  declare_parameter<double>("frequency", 20.0);  // 默认20Hz
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

  // 创建实时性QoS配置
  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));  // 只保留最新1条消息
  qos_profile.best_effort();  // 使用尽力而为传输
  qos_profile.deadline(std::chrono::milliseconds(100));  // 设置截止时间
  qos_profile.lifespan(std::chrono::milliseconds(200));  // 设置消息生命周期

  // 创建发布者
  sensor_publisher_ = this->create_publisher<fake_capture_msgs::msg::CapturedData>("sensor_data", qos_profile);

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
  auto message = fake_capture_msgs::msg::CapturedData();
  
  if (mode_ == "random") {
    // 生成随机数
    message.data = offset_ + distribution_(generator_) * amplitude_ + distribution_(generator_) * noise_;
  } else if (mode_ == "sine") {
    // 生成正弦波
    double time = this->now().seconds();
    message.data = offset_ + amplitude_ * sin(2 * M_PI * 2 * time) + distribution_(generator_) * noise_;
  }

  // 添加时间戳（采集时间）
  message.stamp = this->now();

  sensor_publisher_->publish(message);
  RCLCPP_DEBUG(this->get_logger(), "Published data: %.4f at time: %.6f", message.data, message.stamp.sec + message.stamp.nanosec / 1e9);
  
  // 更新计数器
  ++count_;
}

FakeSensorNode::~FakeSensorNode() {
  RCLCPP_INFO(this->get_logger(), "Fake Sensor Node is shutting down");
  // 添加自定义清理逻辑（如果有）
}

}  // namespace fake_sensor_driver