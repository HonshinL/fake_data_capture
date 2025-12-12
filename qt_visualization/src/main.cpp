#include "qt_visualization/visualization_window.hpp"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <signal.h>

// Global variables for signal handling
QApplication* g_app = nullptr;

// Signal handler for Ctrl+C
void signal_handler(int signum)
{
    if (signum == SIGINT || signum == SIGTERM) {
        RCLCPP_INFO(rclcpp::get_logger("qt_visualization_main"), "Received termination signal, shutting down...");
        
        // Request Qt application to quit
        if (g_app) {
            g_app->quit();
        }
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Initialize Qt application
    QApplication app(argc, argv);
    g_app = &app;
    
    // Register signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Create visualization window using smart pointer
    auto window = std::make_shared<qt_visualization::VisualizationWindow>();
    window->show();
    
    // Run ROS2 spin in a separate thread
    std::thread ros_thread([&]() {
        rclcpp::spin(window->get_node_base_interface());
    });
    
    // Run Qt event loop
    int result = app.exec();
    
    // Cleanup in correct order
    window.reset();  // Ensure window is destroyed first
    rclcpp::shutdown();
    ros_thread.join();
    
    return result;
}