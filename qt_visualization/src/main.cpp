#include "qt_visualization/visualization_window.hpp"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Initialize Qt application
    QApplication app(argc, argv);
    
    // Create visualization window
    VisualizationWindow window;
    window.show();
    
    // Run ROS2 spin in a separate thread
    std::thread ros_thread([&]() {
        rclcpp::spin(window.get_node_base_interface());
    });
    
    // Run Qt event loop
    int result = app.exec();
    
    // Cleanup
    rclcpp::shutdown();
    ros_thread.join();
    
    return result;
}