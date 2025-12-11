#ifndef QT_VISUALIZATION__VISUALIZATION_WINDOW_HPP_
#define QT_VISUALIZATION__VISUALIZATION_WINDOW_HPP_

#include <QMainWindow>
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>
#include <QVBoxLayout>
#include <QTimer>
#include <QSlider>
#include <QLabel>
#include <QHBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <queue>      // 添加queue头文件
#include <mutex>      // 添加mutex头文件

QT_CHARTS_USE_NAMESPACE

class VisualizationWindow : public QMainWindow, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit VisualizationWindow(QWidget *parent = nullptr);
    ~VisualizationWindow() override;

private slots:
    // Slot to update chart with new data
    void updateChart(double value);
    
    // Slot for timer to update UI
    void timerUpdate();
    
    // Slot for zoom slider changes
    void zoomChanged(int value);

private:
    // ROS2 subscription callback
    void sensor_data_callback(const std_msgs::msg::Float64::SharedPtr msg);

    // Initialize ROS2 communication
    void init_ros2();

    // Initialize UI components
    void init_ui();

    // UI components
    QWidget *central_widget_;
    QVBoxLayout *main_layout_;
    QChart *chart_;
    QChartView *chart_view_;
    QLineSeries *series_;
    QValueAxis *x_axis_;
    QValueAxis *y_axis_;
    
    // Zoom controls
    QHBoxLayout *zoom_layout_;
    QLabel *zoom_label_;
    QSlider *zoom_slider_;
    QLabel *zoom_value_label_;

    // ROS2 subscription
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sensor_subscription_;

    // Data buffer for chart (stores all data points)
    QVector<QPointF> all_data_points_;
    double x_counter_;
    // 在private部分添加最大存储长度
    // 调整成员变量声明顺序，使其与构造函数初始化列表顺序一致
    size_t max_storage_points_;  // 最大存储数据点数
    size_t max_data_points_;  // 显示数据点数
    
    // Thread-safe data queue
    std::queue<double> data_queue_;
    std::mutex queue_mutex_;
    
    // Update timer
    QTimer *update_timer_;
};

#endif  // QT_VISUALIZATION__VISUALIZATION_WINDOW_HPP_