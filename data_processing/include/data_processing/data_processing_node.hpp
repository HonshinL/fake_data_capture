#ifndef DATA_PROCESSING__DATA_PROCESSING_NODE_HPP_
#define DATA_PROCESSING__DATA_PROCESSING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "software_fifo.hpp"
#include <QtCore>
#include <thread>
#include <atomic>

class DataProcessingNode : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit DataProcessingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~DataProcessingNode();

signals:
    // Qt signal to emit processed data
    void dataReady(double value);

private:
    // ROS2 subscription callback
    void sensor_data_callback(const std_msgs::msg::Float64::SharedPtr msg);

    // Process data in a separate thread
    void process_data_thread();

    // ROS2 subscription
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sensor_subscription_;

    // Software FIFO for data buffering
    SoftwareFIFO<double> data_fifo_;

    // Thread for data processing
    std::thread processing_thread_;

    // Flag to stop the processing thread
    std::atomic<bool> stop_thread_;
};

#endif  // DATA_PROCESSING__DATA_PROCESSING_NODE_HPP_