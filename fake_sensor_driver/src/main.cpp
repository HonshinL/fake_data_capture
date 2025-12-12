#include "fake_sensor_driver/fake_sensor_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeSensorNode>());
  rclcpp::shutdown();
  return 0;
}