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
#include <fake_capture_msgs/msg/captured_data.hpp>  // 改为正确的消息类型
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

protected:
    // Override close event to handle window closure gracefully
    void closeEvent(QCloseEvent *event) override;

private:
    // Helper methods
    void init_ui();
    void init_ros2();
    void sensor_data_callback(const fake_capture_msgs::msg::CapturedData::SharedPtr msg);  // 改为正确的消息类型
    
    // ROS2 subscription for sensor data
    rclcpp::Subscription<fake_capture_msgs::msg::CapturedData>::SharedPtr sensor_subscription_;  // 改为正确的消息类型
    
    // Thread-safe data queue
    std::queue<double> data_queue_;
    std::mutex queue_mutex_;
    
    // Data storage and counters
    QList<QPointF> all_data_points_;
    double x_counter_;
    size_t max_storage_points_;
    size_t max_data_points_;
    
    // Timer for UI updates
    QTimer *update_timer_;
    
    // UI components
    QChartView *chart_view_;
    QChart *chart_;
    QLineSeries *series_;
    QValueAxis *x_axis_;
    QValueAxis *y_axis_;
    QSlider *zoom_slider_;
    QLabel *zoom_label_;
    QLabel *zoom_value_label_;
    QWidget *central_widget_;
    QVBoxLayout *main_layout_;
    QHBoxLayout *zoom_layout_;
};

#endif  // QT_VISUALIZATION__VISUALIZATION_WINDOW_HPP_