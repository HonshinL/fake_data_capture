#include "data_processing/data_processing_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

DataProcessingNode::DataProcessingNode(const rclcpp::NodeOptions & options)
    : Node("data_processing_node", options),
      data_fifo_(1000),  // FIFO with capacity 1000
      stop_thread_(false)
{
    // Declare parameters
    declare_parameter<double>("processing_frequency", 20.0);  // Default 20Hz

    // Get parameters
    double processing_frequency = get_parameter("processing_frequency").as_double();

    // Create subscription to sensor data
    sensor_subscription_ = this->create_subscription<fake_capture_msgs::msg::CapturedData>(
        "sensor_data",
        10,
        std::bind(&DataProcessingNode::sensor_data_callback, this, std::placeholders::_1));

    // Create publisher for processed data
    processed_data_publisher_ = this->create_publisher<fake_capture_msgs::msg::CapturedData>(
        "processed_sensor_data",
        10);

    // Start processing thread
    processing_thread_ = std::thread(&DataProcessingNode::process_data_thread, this);

    RCLCPP_INFO(this->get_logger(), "Data Processing Node started");
    RCLCPP_INFO(this->get_logger(), "Processing frequency: %.2f Hz", processing_frequency);
}

void DataProcessingNode::process_data_thread()
{
    double process_interval = 1.0 / get_parameter("processing_frequency").as_double();
    auto total_interval = std::chrono::duration<double>(process_interval);
    auto short_sleep = std::chrono::milliseconds(1); // Short sleep interval

    while (rclcpp::ok() && !stop_thread_) {
        DataWithTimestamp data_with_ts;
        
        // Pop data from FIFO if available
        if (data_fifo_.pop(data_with_ts)) {
            // Process the data
            double processed_data = process_data(data_with_ts.data);
            
            // Emit Qt signal with processed data and original timestamp
            emit dataReady(processed_data, data_with_ts.timestamp);
            
            // Publish processed data to ROS2 topic
            auto message = fake_capture_msgs::msg::CapturedData();
            message.data = processed_data;
            message.stamp = data_with_ts.timestamp; // 保留原始时间戳
            processed_data_publisher_->publish(message);
        }

        // Use multiple short sleeps to maintain processing frequency
        // while allowing for responsive termination
        auto start_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>::zero();
        
        while (elapsed < total_interval && rclcpp::ok() && !stop_thread_) {
            std::this_thread::sleep_for(short_sleep);
            elapsed = std::chrono::steady_clock::now() - start_time;
        }
    }
}

// Simple data processing function implementation
double DataProcessingNode::process_data(double raw_data)
{
    // Example: Simple low-pass filter
    static double filtered_value = 0.0;
    double alpha = 0.1;  // Filter coefficient (0-1)
    filtered_value = alpha * raw_data + (1 - alpha) * filtered_value;
    return filtered_value;
}

DataProcessingNode::~DataProcessingNode()
{
    RCLCPP_INFO(this->get_logger(), "Data Processing Node is shutting down");
    
    // Set stop flag
    stop_thread_ = true;
    
    // Wait for processing thread to finish
    if (processing_thread_.joinable()) {
        processing_thread_.join();
        RCLCPP_INFO(this->get_logger(), "Processing thread joined");
    }
    
    RCLCPP_INFO(this->get_logger(), "Data Processing Node shutdown complete");
}

void DataProcessingNode::sensor_data_callback(const fake_capture_msgs::msg::CapturedData::SharedPtr msg)
{
    // 创建包含数据和时间戳的结构体
    DataWithTimestamp data_with_ts;
    data_with_ts.data = msg->data;
    data_with_ts.timestamp = msg->stamp;
    
    // Push received data to FIFO
    if (!data_fifo_.push(data_with_ts)) {
        RCLCPP_WARN(this->get_logger(), "FIFO is full, overwriting oldest data");
    }
}