#ifndef DATA_PROCESSING__DATA_PROCESSING_NODE_HPP_
#define DATA_PROCESSING__DATA_PROCESSING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "fake_capture_msgs/msg/captured_data.hpp"
#include "software_fifo.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <QtCore>
#include <thread>
#include <atomic>
#include <event_bus_cpp/event_bus.hpp>  // 添加EventBus头文件

namespace data_processing {

// 自定义结构体，包含数据和时间戳
struct DataWithTimestamp {
  double data;
  rclcpp::Time timestamp;
};

class DataProcessingNode : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit DataProcessingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~DataProcessingNode();

 signals:
    // Qt signal to emit processed data
    void dataReady(double value, rclcpp::Time timestamp);

private:
    // ROS2 subscription callback
    void sensor_data_callback(const fake_capture_msgs::msg::CapturedData::SharedPtr msg);

    // Process data in a separate thread
    void process_data_thread();

    // Simple data processing function
    double process_data(double raw_data);

    // ROS2 subscription and publisher
    rclcpp::Subscription<fake_capture_msgs::msg::CapturedData>::SharedPtr sensor_subscription_;
    rclcpp::Publisher<fake_capture_msgs::msg::CapturedData>::SharedPtr processed_data_publisher_;

    // Software FIFO for data buffering (包含时间戳)
    SoftwareFIFO<DataWithTimestamp> data_fifo_;

    // Thread for data processing
    std::thread processing_thread_;

    // Flag to stop the processing thread
    std::atomic<bool> stop_thread_;
};

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(data_processing::DataProcessingNode)

}  // namespace data_processing

#endif  // DATA_PROCESSING__DATA_PROCESSING_NODE_HPP_