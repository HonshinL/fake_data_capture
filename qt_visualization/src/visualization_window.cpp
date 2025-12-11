#include "qt_visualization/visualization_window.hpp"
#include <QApplication>  // 添加QApplication头文件
#include <chrono>
#include <iostream>
#include <QCloseEvent>

using namespace std::chrono_literals;

VisualizationWindow::VisualizationWindow(QWidget *parent)
    : QMainWindow(parent),
      rclcpp::Node("visualization_window"),
      x_counter_(0.0),
      max_storage_points_(10000),  // 最大存储10000个点
      max_data_points_(500)  // 默认显示500个点
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
    resize(1000, 600);

    // Create central widget and main layout
    central_widget_ = new QWidget(this);
    setCentralWidget(central_widget_);
    main_layout_ = new QVBoxLayout(central_widget_);

    // Create chart and chart view
    chart_ = new QChart();
    chart_view_ = new QChartView(chart_);
    chart_view_->setRenderHint(QPainter::Antialiasing);

    // Create data series
    series_ = new QLineSeries();
    series_->setName("Sensor Data");
    chart_->addSeries(series_);

    // Create axes
    x_axis_ = new QValueAxis();
    x_axis_->setTitleText("Time (s)");
    x_axis_->setRange(0, max_data_points_);  // Start from time 0
    x_axis_->setTickCount(11);
    x_axis_->setLabelFormat("%.1f");  // Show one decimal place for time

    y_axis_ = new QValueAxis();
    y_axis_->setTitleText("Value");
    y_axis_->setRange(0, 10);  // Initial range, will auto-adjust
    y_axis_->setTickCount(11);

    // Add axes to chart
    chart_->addAxis(x_axis_, Qt::AlignBottom);
    chart_->addAxis(y_axis_, Qt::AlignLeft);
    series_->attachAxis(x_axis_);
    series_->attachAxis(y_axis_);

    // Add chart view to main layout
    main_layout_->addWidget(chart_view_);

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
    update_timer_->start(50);  // Update UI every 50ms
}

void VisualizationWindow::init_ros2()
{
    // Create subscription to sensor_data topic
    sensor_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "sensor_data",
        10,
        std::bind(&VisualizationWindow::sensor_data_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Visualization window initialized");
}

void VisualizationWindow::sensor_data_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Add data to thread-safe queue
    std::lock_guard<std::mutex> lock(queue_mutex_);
    data_queue_.push(msg->data);
}

void VisualizationWindow::timerUpdate()
{
    // Process all pending data points
    while (true) {
        double value;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (data_queue_.empty()) {
                break;
            }
            value = data_queue_.front();
            data_queue_.pop();
        }
        
        // Update chart
        updateChart(value);
    }
}

void VisualizationWindow::updateChart(double value)
{
    // Add new data point at current time (x_counter_ starts from 0)
    all_data_points_.append(QPointF(x_counter_, value));
    x_counter_ += 1.0;  // Increment time counter (assuming 1 unit = 1 second)

    // Control memory usage: remove oldest data if we exceed max storage
    if (static_cast<size_t>(all_data_points_.size()) > max_storage_points_) {
        all_data_points_.removeFirst();
    }

    // Determine which data points to display based on max_data_points_
    QVector<QPointF> display_points;
    int total_points = all_data_points_.size();
    int start_index = std::max(0, total_points - static_cast<int>(max_data_points_));
    
    for (int i = start_index; i < total_points; ++i) {
        display_points.append(all_data_points_[i]);
    }

    // Update series data
    series_->replace(display_points);

    // Auto-adjust axes
    if (display_points.size() > 0) {
        // Adjust Y axis range to show all displayed data
        double y_min = display_points.first().y();
        double y_max = display_points.first().y();
        
        for (const auto &point : display_points) {
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
        if (static_cast<size_t>(total_points) < max_data_points_) {
            // Still filling the window, show from time 0
            x_axis_->setRange(0, max_data_points_);
        } else {
            // Window is full, show last max_data_points_ seconds
            double x_end = x_counter_;
            double x_start = x_end - max_data_points_;
            x_axis_->setRange(x_start, x_end);
        }
    }
}

void VisualizationWindow::zoomChanged(int value)
{
    // Update max_data_points_ with slider value
    max_data_points_ = static_cast<size_t>(value);
    
    // Update slider label
    zoom_value_label_->setText(QString("%1 points").arg(value));
    
    // Determine which data points to display based on new max_data_points_
    QVector<QPointF> display_points;
    int total_points = all_data_points_.size();
    int start_index = std::max(0, total_points - static_cast<int>(max_data_points_));
    
    for (int i = start_index; i < total_points; ++i) {
        display_points.append(all_data_points_[i]);
    }
    
    // Update chart to reflect new zoom level
    series_->replace(display_points);
    
    // Update X axis range
    if (static_cast<size_t>(total_points) < max_data_points_) {
        // Still filling the window, show from time 0
        x_axis_->setRange(0, max_data_points_);
    } else {
        // Window is full, show last max_data_points_ seconds
        double x_end = x_counter_;
        double x_start = x_end - max_data_points_;
        x_axis_->setRange(x_start, x_end);
    }
}