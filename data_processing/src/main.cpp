#include "rclcpp/rclcpp.hpp"
#include "data_processing/data_processing_node.hpp"
#include <QtCore>

int main(int argc, char * argv[])
{
    // Initialize Qt
    QCoreApplication app(argc, argv);

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create and spin the node
    auto node = std::make_shared<DataProcessingNode>();
    
    // Create a thread to spin the node
    std::thread ros_thread([&node]() {
        rclcpp::spin(node);
    });

    // Run Qt event loop
    int result = app.exec();

    // Clean up
    rclcpp::shutdown();
    ros_thread.join();

    return result;
}