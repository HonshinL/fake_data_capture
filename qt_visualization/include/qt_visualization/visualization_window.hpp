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
#include <QComboBox>
#include <event_bus_cpp/event_bus.hpp>

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
    void onRosDataReceived(double value, double timestamp_us);

private slots:
    // Slot to update data points with new data
    void updateDataPoints(double value, double latency, bool is_ros);
    
    // Slot for timer to update UI
    void updateCharts();
    
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
    void init_event_bus();
    
    // 通用的图表更新辅助方法
    void updateChartSeries(QLineSeries* series, QValueAxis* y_axis, 
                          const QVector<QPointF>& data_points, 
                          double padding_factor, double min_padding);
    
    // Data storage and counters
    QList<QPointF> data_points_;
    QList<QPointF> latency_points_;  // 合并后的延迟数据点
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
    QLineSeries *latency_series_;  // 合并后的延迟曲线
    QLineSeries *average_latency_series_;  // 合并后的平均延迟曲线
    QValueAxis *latency_x_axis_;
    QValueAxis *latency_y_axis_;
    
    // UI components for average latency display
    QLabel *ros_average_latency_label_;  // ROS平均延迟标签
    QLabel *eventbus_average_latency_label_;  // EventBus平均延迟标签
    double total_latency_;  // 合并后的总延迟
    int latency_point_count_;  // 合并后的延迟数据点数量
    
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