#ifndef FAKE_SENSOR_DRIVER__FAKE_SENSOR_NODE_HPP_
#define FAKE_SENSOR_DRIVER__FAKE_SENSOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "fake_capture_msgs/msg/captured_data.hpp"  // 使用带有时间戳的消息类型
#include <random>

class FakeSensorNode : public rclcpp::Node
{
public:
  FakeSensorNode();
  ~FakeSensorNode();

private:
  void timer_callback();

  rclcpp::Publisher<fake_capture_msgs::msg::CapturedData>::SharedPtr sensor_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::string mode_;
  double amplitude_;
  double offset_;
  double noise_;
  
  std::default_random_engine generator_;
  std::uniform_real_distribution<double> distribution_{-1.0, 1.0};
  size_t count_ = 0;
};

#endif  // FAKE_SENSOR_DRIVER__FAKE_SENSOR_NODE_HPP_