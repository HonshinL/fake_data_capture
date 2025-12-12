#include "qt_visualization/visualization_window.hpp"
#include <QApplication>  // 添加QApplication头文件
#include <chrono>
#include <iostream>
#include <QCloseEvent>
#include <fake_capture_msgs/msg/captured_data.hpp>  // 使用带有时间戳的消息类型
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace qt_visualization {

VisualizationWindow::VisualizationWindow(const rclcpp::NodeOptions & options, QWidget *parent)
    : QMainWindow(parent),
      rclcpp::Node("visualization_window", options),
      x_counter_(0.0),
      max_storage_points_(10000),  // 最大存储10000个点
      max_data_points_(200)  // 默认显示500个点
{
    init_ui();
    init_ros2();
}

VisualizationWindow::~VisualizationWindow()
{
    RCLCPP_INFO(this->get_logger(), "VisualizationWindow destructor called");
    
    // Stop and delete update timer
    if (update_timer_) {
        update_timer_->stop();
        delete update_timer_;
        update_timer_ = nullptr;
    }
    
    // The ROS2 subscription will be automatically destroyed when the node is deleted
    RCLCPP_INFO(this->get_logger(), "VisualizationWindow destructor completed");
}

// Override closeEvent to handle window close gracefully
void VisualizationWindow::closeEvent(QCloseEvent *event)
{
    RCLCPP_INFO(this->get_logger(), "Visualization window closing...");
    
    // Stop timer
    if (update_timer_) {
        update_timer_->stop();
    }
    
    // Accept the close event
    event->accept();
    
    // Request Qt application to quit, ensuring entire application exits
    QApplication::quit();
    
    // Let Qt handle the rest of the shutdown
    QMainWindow::closeEvent(event);
}

void VisualizationWindow::init_ui()
{
    // Set window properties
    setWindowTitle("Sensor Data Visualization");
    resize(1800, 1200);  // 增加窗口初始大小以容纳两个图表

    // Create central widget and main layout
    central_widget_ = new QWidget(this);
    setCentralWidget(central_widget_);
    main_layout_ = new QVBoxLayout(central_widget_);

    // Create main data chart and chart view
    chart_ = new QChart();
    chart_view_ = new QChartView(chart_);
    chart_view_->setRenderHint(QPainter::Antialiasing);

    // Create main data series
    series_ = new QLineSeries();
    series_->setName("Sensor Data");
    chart_->addSeries(series_);

    // Create main data axes
    x_axis_ = new QValueAxis();
    x_axis_->setTitleText("Time (s)");
    x_axis_->setRange(0, max_data_points_);  // Start from time 0
    x_axis_->setTickCount(11);
    x_axis_->setLabelFormat("%.1f");  // Show one decimal place for time
    x_axis_->setLabelsFont(QFont("Arial", 8));  // 设置X轴数字显示大小

    y_axis_ = new QValueAxis();
    y_axis_->setTitleText("Value");
    y_axis_->setRange(0, 10);  // Initial range, will auto-adjust
    y_axis_->setTickCount(11);
    y_axis_->setLabelsFont(QFont("Arial", 8));  // 设置Y轴数字显示大小

    // Add main data axes to chart
    chart_->addAxis(x_axis_, Qt::AlignBottom);
    chart_->addAxis(y_axis_, Qt::AlignLeft);
    series_->attachAxis(x_axis_);
    series_->attachAxis(y_axis_);

    // Add main data chart view to main layout
    main_layout_->addWidget(chart_view_);
    
    // Create latency chart and chart view
    latency_chart_ = new QChart();
    latency_chart_view_ = new QChartView(latency_chart_);
    latency_chart_view_->setRenderHint(QPainter::Antialiasing);

    // Create latency data series
    latency_series_ = new QLineSeries();
    latency_series_->setName("Latency (ms)");
    latency_chart_->addSeries(latency_series_);

    // Create latency data axes
    latency_x_axis_ = new QValueAxis();
    latency_x_axis_->setTitleText("Time (s)");
    latency_x_axis_->setRange(0, max_data_points_);  // Start from time 0
    latency_x_axis_->setTickCount(11);
    latency_x_axis_->setLabelFormat("%.1f");  // Show one decimal place for time
    latency_x_axis_->setLabelsFont(QFont("Arial", 8));  // 设置X轴数字显示大小

    latency_y_axis_ = new QValueAxis();
    latency_y_axis_->setTitleText("Latency (ms)");
    latency_y_axis_->setRange(0, 100);  // Initial range for latency (milliseconds)
    latency_y_axis_->setTickCount(11);
    latency_y_axis_->setLabelsFont(QFont("Arial", 8));  // 设置Y轴数字显示大小

    // Add latency data axes to chart
    latency_chart_->addAxis(latency_x_axis_, Qt::AlignBottom);
    latency_chart_->addAxis(latency_y_axis_, Qt::AlignLeft);
    latency_series_->attachAxis(latency_x_axis_);
    latency_series_->attachAxis(latency_y_axis_);

    // Add latency chart view to main layout
    main_layout_->addWidget(latency_chart_view_);

    // Create zoom controls
    zoom_layout_ = new QHBoxLayout();
    zoom_label_ = new QLabel("Display Length:");
    zoom_slider_ = new QSlider(Qt::Horizontal);
    zoom_slider_->setRange(100, max_storage_points_);  // 限制缩放范围不超过存储范围
    zoom_slider_->setValue(max_data_points_);
    zoom_value_label_ = new QLabel(QString("%1 points").arg(max_data_points_));

    // Add zoom controls to layout
    zoom_layout_->addWidget(zoom_label_);
    zoom_layout_->addWidget(zoom_slider_);
    zoom_layout_->addWidget(zoom_value_label_);
    main_layout_->addLayout(zoom_layout_);

    // Connect zoom slider signal
    connect(zoom_slider_, &QSlider::valueChanged, this, &VisualizationWindow::zoomChanged);

    // Create update timer
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &VisualizationWindow::timerUpdate);
    // 将UI更新定时器频率降低到1秒
    update_timer_->start(1000);  // Update UI every 1 second
    
}

void VisualizationWindow::init_ros2()
{
    // 创建实时性QoS配置
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));  // 只保留最新1条消息
    qos_profile.best_effort();  // 使用尽力而为传输
    qos_profile.deadline(std::chrono::milliseconds(100));  // 设置截止时间
    qos_profile.lifespan(std::chrono::milliseconds(200));  // 设置消息生命周期

    // Create subscription to processed sensor data topic instead of raw data
    sensor_subscription_ = this->create_subscription<fake_capture_msgs::msg::CapturedData>(
    "processed_sensor_data",
    qos_profile,
    std::bind(&VisualizationWindow::sensor_data_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Visualization window initialized");
}

void VisualizationWindow::sensor_data_callback(const fake_capture_msgs::msg::CapturedData::SharedPtr msg)
{
    // 创建包含数据和时间戳的结构体
    DataWithTimestamp data_with_ts;
    data_with_ts.data = msg->data;
    data_with_ts.timestamp = msg->stamp;
    
    // 计算数据采集和显示之间的时间差
    rclcpp::Time current_time = this->now();
    rclcpp::Duration time_diff = current_time - data_with_ts.timestamp;
    
    // 将时间差转换为毫秒
    double latency_ms = time_diff.nanoseconds() / 1e6;
    
    // 将时间差输出到终端（毫秒）
    std::cout << "数据采集和显示之间的时间差: " << latency_ms << " 毫秒" << std::endl;
    
    // 直接更新图表，同时传递原始数据和延迟数据
    updateChart(data_with_ts.data, latency_ms);
}

void VisualizationWindow::updateChart(double value, double latency)
{
    // Add new data point at current time (x_counter_ starts from 0)
    all_data_points_.append(QPointF(x_counter_, value));
    
    // Add new latency point at current time
    all_latency_points_.append(QPointF(x_counter_, latency));
    
    x_counter_ += 1.0;  // Increment time counter (assuming 1 unit = 1 second)

    // Control memory usage: remove oldest data if we exceed max storage
    if (static_cast<size_t>(all_data_points_.size()) > max_storage_points_) {
        all_data_points_.removeFirst();
    }
    
    if (static_cast<size_t>(all_latency_points_.size()) > max_storage_points_) {
        all_latency_points_.removeFirst();
    }

    // Determine which data points to display based on max_data_points_
    QVector<QPointF> display_data_points;
    QVector<QPointF> display_latency_points;
    
    int total_data_points = all_data_points_.size();
    int start_index = std::max(0, total_data_points - static_cast<int>(max_data_points_));
    
    for (int i = start_index; i < total_data_points; ++i) {
        display_data_points.append(all_data_points_[i]);
        display_latency_points.append(all_latency_points_[i]);
    }

    // Update series data
    series_->replace(display_data_points);
    latency_series_->replace(display_latency_points);

    // Auto-adjust axes for main data chart
    if (display_data_points.size() > 0) {
        // Adjust Y axis range to show all displayed data
        double y_min = display_data_points.first().y();
        double y_max = display_data_points.first().y();
        
        for (const auto &point : display_data_points) {
            y_min = std::min(y_min, point.y());
            y_max = std::max(y_max, point.y());
        }
        
        // Add some padding
        double y_padding = (y_max - y_min) * 0.1;
        if (y_padding == 0) {
            y_padding = 0.5;  // Minimum padding if all values are the same
        }
        
        y_axis_->setRange(y_min - y_padding, y_max + y_padding);
        
        // Adjust X axis range based on current display data
        if (static_cast<size_t>(total_data_points) < max_data_points_) {
            // Still filling the window, show from time 0
            x_axis_->setRange(0, max_data_points_);
            latency_x_axis_->setRange(0, max_data_points_);
        } else {
            // Window is full, show last max_data_points_ seconds
            double x_end = x_counter_;
            double x_start = x_end - max_data_points_;
            x_axis_->setRange(x_start, x_end);
            latency_x_axis_->setRange(x_start, x_end);
        }
    }
    
    // Auto-adjust axes for latency chart
    if (display_latency_points.size() > 0) {
        // Adjust Y axis range to show all displayed latency data
        double latency_min = display_latency_points.first().y();
        double latency_max = display_latency_points.first().y();
        
        for (const auto &point : display_latency_points) {
            latency_min = std::min(latency_min, point.y());
            latency_max = std::max(latency_max, point.y());
        }
        
        // Add some padding
        double latency_padding = (latency_max - latency_min) * 0.1;
        if (latency_padding == 0) {
            latency_padding = 5.0;  // Minimum padding for latency
        }
        
        latency_y_axis_->setRange(latency_min - latency_padding, latency_max + latency_padding);
    }
}

void VisualizationWindow::zoomChanged(int value)
{
    // Update max_data_points_ with slider value
    max_data_points_ = static_cast<size_t>(value);
    
    // Update slider label
    zoom_value_label_->setText(QString("%1 points").arg(value));
    
    // Determine which data points to display based on new max_data_points_
    QVector<QPointF> display_data_points;
    QVector<QPointF> display_latency_points;
    
    int total_data_points = all_data_points_.size();
    int start_index = std::max(0, total_data_points - static_cast<int>(max_data_points_));
    
    for (int i = start_index; i < total_data_points; ++i) {
        display_data_points.append(all_data_points_[i]);
        display_latency_points.append(all_latency_points_[i]);
    }
    
    // Update charts to reflect new zoom level
    series_->replace(display_data_points);
    latency_series_->replace(display_latency_points);
    
    // Update X axis ranges
    if (static_cast<size_t>(total_data_points) < max_data_points_) {
        // Still filling the window, show from time 0
        x_axis_->setRange(0, max_data_points_);
        latency_x_axis_->setRange(0, max_data_points_);
    } else {
        // Window is full, show last max_data_points_ seconds
        double x_end = x_counter_;
        double x_start = x_end - max_data_points_;
        x_axis_->setRange(x_start, x_end);
        latency_x_axis_->setRange(x_start, x_end);
    }
}

void VisualizationWindow::timerUpdate()
{
    // 不再处理数据队列，仅保持定时器运行以维持节点活动
    // 可以添加一些定期维护任务，如清理内存等
}
}