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
#include <rclcpp_components/register_node_macro.hpp>
#include <fake_capture_msgs/msg/captured_data.hpp>  // 使用带有时间戳的消息类型
#include <queue>      // 添加queue头文件
#include <mutex>      // 添加mutex头文件

QT_CHARTS_USE_NAMESPACE

namespace qt_visualization {

// 自定义结构体，包含数据和时间戳
struct DataWithTimestamp {
  double data;
  rclcpp::Time timestamp;
};

class VisualizationWindow : public QMainWindow, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit VisualizationWindow(const rclcpp::NodeOptions & options = rclcpp::NodeOptions(), QWidget *parent = nullptr);
    ~VisualizationWindow() override;

private slots:
    // Slot to update charts with new data
    void updateChart(double value, double latency);
    
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
    void sensor_data_callback(const fake_capture_msgs::msg::CapturedData::SharedPtr msg);  // 使用带有时间戳的消息类型
    
    // ROS2 subscription for sensor data
    rclcpp::Subscription<fake_capture_msgs::msg::CapturedData>::SharedPtr sensor_subscription_;  // 使用带有时间戳的消息类型
    
    // Thread-safe data queue (包含时间戳)
    std::queue<DataWithTimestamp> data_queue_;
    std::mutex queue_mutex_;
    
    // Data storage and counters
    QList<QPointF> all_data_points_;
    QList<QPointF> all_latency_points_;  // 存储延迟数据点
    double x_counter_;
    size_t max_storage_points_;
    size_t max_data_points_;
    
    // Timer for UI updates
    QTimer *update_timer_;
    
    // UI components for main data chart
    QChartView *chart_view_;
    QChart *chart_;
    QLineSeries *series_;
    QValueAxis *x_axis_;
    QValueAxis *y_axis_;
    
    // UI components for latency chart
    QChartView *latency_chart_view_;
    QChart *latency_chart_;
    QLineSeries *latency_series_;
    QValueAxis *latency_x_axis_;
    QValueAxis *latency_y_axis_;
    
    // UI components for average latency display
    QLabel *average_latency_label_;  // 显示平均延迟的标签
    QLineSeries *average_latency_series_;  // 平均延迟的曲线
    double total_latency_;  // 总延迟，用于计算平均值
    int latency_point_count_;  // 延迟数据点数量
    
    // Zoom controls
    QSlider *zoom_slider_;
    QLabel *zoom_label_;
    QLabel *zoom_value_label_;
    QWidget *central_widget_;
    QVBoxLayout *main_layout_;
    QHBoxLayout *zoom_layout_;
};

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(qt_visualization::VisualizationWindow)

}  // namespace qt_visualization

#endif  // QT_VISUALIZATION__VISUALIZATION_WINDOW_HPP_