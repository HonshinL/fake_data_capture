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
#include <QComboBox>  // 添加QComboBox头文件
#include <event_bus_cpp/event_bus.hpp>  // 添加EventBus头文件

QT_CHARTS_USE_NAMESPACE

namespace qt_visualization {

class VisualizationWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit VisualizationWindow(QWidget *parent = nullptr);
    ~VisualizationWindow() override;

public Q_SLOTS:
    // Slot to receive data from ROS node
    void onRosDataReceived(double value, double latency);

private slots:
    // Slot to update charts with new data
    void updateChart(double value, double latency, bool is_ros);
    
    // Slot for timer to update UI
    void timerUpdate();
    
    // Slot for zoom slider changes
    void zoomChanged(int value);
    
    // Slot for EventBus callback
    void onEventBusDataReceived(const QVariantMap& event_data);
    
    // Slot for communication mode selection changes
    void onCommunicationModeChanged(int index);

protected:
    // Override close event to handle window closure gracefully
    void closeEvent(QCloseEvent *event) override;

private:
    // Helper methods
    void init_ui();
    void init_event_bus();  // 添加EventBus初始化方法
    
    // Data storage and counters
    QList<QPointF> all_data_points_;
    QList<QPointF> all_ros_latency_points_;  // ROS延迟数据点
    QList<QPointF> all_eventbus_latency_points_;  // EventBus延迟数据点
    double x_counter_;
    size_t max_storage_points_;
    size_t max_data_points_;
    
    // Communication mode tracking
    std::string communication_mode_;  // "ros" 或 "eventbus"
    
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
    QLineSeries *ros_latency_series_;  // ROS延迟曲线
    QLineSeries *eventbus_latency_series_;  // EventBus延迟曲线
    QLineSeries *ros_average_latency_series_;  // ROS平均延迟曲线
    QLineSeries *eventbus_average_latency_series_;  // EventBus平均延迟曲线
    QValueAxis *latency_x_axis_;
    QValueAxis *latency_y_axis_;
    
    // UI components for average latency display
    QLabel *ros_average_latency_label_;  // ROS平均延迟标签
    QLabel *eventbus_average_latency_label_;  // EventBus平均延迟标签
    double ros_total_latency_;  // ROS总延迟
    double eventbus_total_latency_;  // EventBus总延迟
    int ros_latency_point_count_;  // ROS延迟数据点数量
    int eventbus_latency_point_count_;  // EventBus延迟数据点数量
    
    // UI components for communication mode selection
    QComboBox *communication_mode_combo_;  // 通信方式选择下拉框
    QHBoxLayout *communication_mode_layout_;  // 通信方式选择布局
    
    // Zoom controls
    QSlider *zoom_slider_;
    QLabel *zoom_label_;
    QLabel *zoom_value_label_;
    QWidget *central_widget_;
    QVBoxLayout *main_layout_;
    QHBoxLayout *zoom_layout_;
    QHBoxLayout *latency_labels_layout_;  // 延迟标签布局
    
    // EventBus subscription ID
    EventBus::HandlerId eventbus_subscription_id_;
};

}  // namespace qt_visualization

#endif  // QT_VISUALIZATION__VISUALIZATION_WINDOW_HPP_