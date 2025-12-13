#include <rclcpp/rclcpp.hpp>
#include <fake_capture_msgs/msg/captured_data.hpp>

#include <QObject>
#include "qt_visualization/qt_sensor_data_node.hpp"
#include <chrono>
#include <iostream>

namespace qt_visualization {

SensorDataNode::SensorDataNode(const rclcpp::NodeOptions & options)
    : QObject(), rclcpp::Node("qt_sensor_data_node", options)
{
    // 创建实时性QoS配置
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));  // 只保留最新1条消息
    qos_profile.best_effort();  // 使用尽力而为传输
    qos_profile.deadline(std::chrono::milliseconds(100));  // 设置截止时间
    qos_profile.lifespan(std::chrono::milliseconds(200));  // 设置消息生命周期

    // Create subscription to processed sensor data topic
    sensor_subscription_ = this->create_subscription<fake_capture_msgs::msg::CapturedData>(
        "processed_sensor_data",
        qos_profile,
        std::bind(&SensorDataNode::sensor_data_callback, this, std::placeholders::_1));

    // 使用std::cout替代RCLCPP_INFO，避免在析构时可能的问题
    std::cout << "Sensor Data Node initialized" << std::endl;
}

SensorDataNode::~SensorDataNode()
{
    // 不需要显式释放sensor_subscription_，智能指针会处理
    // 避免在析构时使用ROS2日志功能，因为ROS2系统可能已经关闭
}

void SensorDataNode::sensor_data_callback(const fake_capture_msgs::msg::CapturedData::SharedPtr msg)
{
    // 将ROS时间戳转换为微秒级精度
    double timestamp_us = msg->stamp.sec * 1e6 + msg->stamp.nanosec / 1e3;
    
    // 发出Qt信号，传递原始数据和时间戳
    Q_EMIT dataReceived(msg->data, timestamp_us);
}

}  // namespace qt_visualization