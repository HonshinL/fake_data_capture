#include "qt_visualization/visualization_window.hpp"
#include "qt_visualization/sensor_data_node.hpp"
#include "data_processing/data_processing_node.hpp"  // 包含DataProcessingNode头文件
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <signal.h>
#include <iostream>
#include <memory>

// Global variables for signal handling
QApplication* g_app = nullptr;

// Signal handler for Ctrl+C
void signal_handler(int signum)
{
    if (signum == SIGINT || signum == SIGTERM) {
        std::cout << "Received termination signal, shutting down..." << std::endl;
        
        // Request Qt application to quit
        if (g_app) {
            g_app->quit();
        }
    }
}

int main(int argc, char **argv)
{
    // Initialize Qt application first
    QApplication app(argc, argv);
    g_app = &app;
    
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Register signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Create ROS nodes
    auto data_processing_node = std::make_shared<data_processing::DataProcessingNode>();
    auto sensor_data_node = std::make_shared<qt_visualization::SensorDataNode>();
    
    // Create visualization window
    qt_visualization::VisualizationWindow window;
    window.show();
    
    // Connect ROS node signal to window slot
    QObject::connect(sensor_data_node.get(), &qt_visualization::SensorDataNode::dataReceived,
                     &window, &qt_visualization::VisualizationWindow::onRosDataReceived);
    
    // Run both nodes in a single thread with periodic checking
    bool is_running = true;
    std::thread ros_thread([&]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(data_processing_node);
        executor.add_node(sensor_data_node);
        
        // Use a timeout to periodically check if we should exit
        while (is_running && rclcpp::ok()) {
            executor.spin_once(std::chrono::milliseconds(100));
        }
    });
    
    // Run Qt event loop
    int result = app.exec();
    
    // Cleanup in correct order
    std::cout << "Qt event loop exited, cleaning up..." << std::endl;
    
    // Set flag to stop ROS thread
    is_running = false;
    
    // 1. 首先调用rclcpp::shutdown()关闭ROS2系统
    if (rclcpp::ok()) {
        rclcpp::shutdown();
        std::cout << "ROS2 shutdown completed" << std::endl;
    }
    
    // 2. 等待ros_thread结束
    if (ros_thread.joinable()) {
        std::cout << "Waiting for ROS thread to exit..." << std::endl;
        ros_thread.join();
        std::cout << "ROS thread exited" << std::endl;
    }
    
    // 3. 最后将nodes置为空，释放节点资源
    data_processing_node.reset();
    sensor_data_node.reset();
    std::cout << "All resources released, exiting..." << std::endl;
    
    return result;
}