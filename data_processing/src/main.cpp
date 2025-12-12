#include "rclcpp/rclcpp.hpp"
#include "data_processing/data_processing_node.hpp"
#include <QtCore>
#include <signal.h>

// Global variables for signal handling
std::shared_ptr<data_processing::DataProcessingNode> g_node = nullptr;
QCoreApplication* g_app = nullptr;

// Signal handler for Ctrl+C
void signal_handler(int signum)
{
    if (signum == SIGINT || signum == SIGTERM) {
        RCLCPP_INFO(rclcpp::get_logger("data_processing_main"), "Received termination signal, shutting down...");
        
        // Request Qt application to quit (this will trigger the main cleanup sequence)
        if (g_app) {
            g_app->quit();
        }
    }
}

int main(int argc, char * argv[])
{
    // Initialize Qt
    QCoreApplication app(argc, argv);
    g_app = &app;

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create and spin the node
    g_node = std::make_shared<data_processing::DataProcessingNode>();
    
    // Create a thread to spin the node
    std::thread ros_thread([&]() {
        rclcpp::spin(g_node);
    });

    // Register signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Run Qt event loop
    int result = app.exec();

    // Clean up - correct order is crucial!
    // 1. First reset the node to ensure proper destruction and resource cleanup
    g_node.reset();
    
    // 2. Then shutdown ROS2 system
    rclcpp::shutdown();
    
    // 3. Wait for ROS thread to finish
    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    return result;
}