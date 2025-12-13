#ifndef QT_VISUALIZATION__SENSOR_DATA_NODE_HPP_
#define QT_VISUALIZATION__SENSOR_DATA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <fake_capture_msgs/msg/captured_data.hpp>

#include <QObject>

namespace qt_visualization {

class SensorDataNode : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit SensorDataNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~SensorDataNode() override;

Q_SIGNALS:
    // Qt signal to emit sensor data with latency
    void dataReceived(double value, double latency);

private:
    // ROS2 subscription callback
    void sensor_data_callback(const fake_capture_msgs::msg::CapturedData::SharedPtr msg);
    
    // ROS2 subscription for sensor data
    rclcpp::Subscription<fake_capture_msgs::msg::CapturedData>::SharedPtr sensor_subscription_;
};

// 移除组件注册宏，因为我们直接创建节点实例

}  // namespace qt_visualization

#endif  // QT_VISUALIZATION__SENSOR_DATA_NODE_HPP_