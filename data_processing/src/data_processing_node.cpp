#include "data_processing/data_processing_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "fake_capture_msgs/msg/captured_data.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <chrono>
#include <iostream>
#include <event_bus_cpp/event_bus.hpp>  // 添加EventBus头文件

namespace data_processing {

DataProcessingNode::DataProcessingNode(const rclcpp::NodeOptions & options)
    : QObject(), Node("data_processing_node", options),
      data_fifo_(100),  // 减小FIFO容量以减少延迟
      stop_thread_(false)
{
    // Declare parameters
    this->declare_parameter("processing_frequency", 20.0);
    this->declare_parameter("alpha", 0.1);  // Low-pass filter parameter

    // Create QoS profile for best effort communication with consistent parameters
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
        .best_effort()
        .deadline(std::chrono::milliseconds(100))
        .lifespan(std::chrono::milliseconds(200));

    // Create subscription to sensor data
    sensor_subscription_ = this->create_subscription<fake_capture_msgs::msg::CapturedData>(
        "sensor_data",
        qos_profile,
        std::bind(&DataProcessingNode::sensor_data_callback, this, std::placeholders::_1));

    // Create publisher for processed data
    processed_data_publisher_ = this->create_publisher<fake_capture_msgs::msg::CapturedData>(
        "processed_sensor_data",
        qos_profile);

    // Start processing thread
    processing_thread_ = std::thread(&DataProcessingNode::process_data_thread, this);

    RCLCPP_INFO(this->get_logger(), "Data Processing Node started");
    RCLCPP_INFO(this->get_logger(), "Using event-driven processing mode");
}

DataProcessingNode::~DataProcessingNode()
{
    // Set stop flag and wait for thread to finish
    stop_thread_ = true;
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "Data Processing Node stopped");
}

void DataProcessingNode::sensor_data_callback(const fake_capture_msgs::msg::CapturedData::SharedPtr msg)
{
    // 将接收到的数据和时间戳存入FIFO
    DataWithTimestamp data_with_ts;
    data_with_ts.data = msg->data;
    data_with_ts.timestamp = msg->stamp;
    
    bool pushed = data_fifo_.push(data_with_ts);
    if (!pushed) {
        RCLCPP_WARN(this->get_logger(), "FIFO is full, oldest data overwritten");
    }
}

void DataProcessingNode::process_data_thread()
{
    DataWithTimestamp data_with_ts;
    
    while (rclcpp::ok() && !stop_thread_) {
        // 使用阻塞方式等待新数据，超时时间为100ms
        if (data_fifo_.pop_blocking(data_with_ts)) {
            // 处理数据
            double processed_data = process_data(data_with_ts.data);
            
            // 发布处理后的数据
            auto message = fake_capture_msgs::msg::CapturedData();
            message.data = processed_data;
            message.stamp = data_with_ts.timestamp; // 保留原始时间戳
            processed_data_publisher_->publish(message);
            
            // 同时使用EventBus发布数据
            QVariantMap event_data;
            event_data["data"] = processed_data;
            event_data["timestamp"] = QDateTime::fromMSecsSinceEpoch(data_with_ts.timestamp.nanoseconds() / 1e6);
            // 使用EVENT_BUS宏替换EventBus::instance()
            EVENT_BUS.publish("processed_sensor_data", event_data);
            
            // 发送Qt信号
            emit dataReady(processed_data, data_with_ts.timestamp);
        }
    }
}

double DataProcessingNode::process_data(double raw_data)
{
    // 获取滤波参数
    double alpha = this->get_parameter("alpha").as_double();
    
    // 简单低通滤波
    static double filtered_data = raw_data;  // 静态变量保存上次滤波结果
    filtered_data = alpha * raw_data + (1.0 - alpha) * filtered_data;
    
    return filtered_data;
}

}  // namespace data_processing

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(data_processing::DataProcessingNode)