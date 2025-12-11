#ifndef FAKE_SENSOR_DRIVER__FAKE_SENSOR_NODE_HPP_
#define FAKE_SENSOR_DRIVER__FAKE_SENSOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <random>

class FakeSensorNode : public rclcpp::Node
{
public:
  FakeSensorNode();
  ~FakeSensorNode() = default;

private:
  void timer_callback();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sensor_publisher_;
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