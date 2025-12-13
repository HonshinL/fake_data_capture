#include "qt_visualization/visualization_window.hpp"
#include <QApplication>
#include <QDateTime>
#include <chrono>
#include <iostream>
#include <QCloseEvent>
#include <event_bus_cpp/event_bus.hpp>

using namespace std::chrono_literals;

namespace qt_visualization {

VisualizationWindow::VisualizationWindow(QWidget *parent)
    : QMainWindow(parent),
      x_counter_(0.0),
      max_storage_points_(10000),
      max_data_points_(200),
      communication_mode_("ros"),
      ros_total_latency_(0.0),
      eventbus_total_latency_(0.0),
      ros_latency_point_count_(0),
      eventbus_latency_point_count_(0),
      eventbus_subscription_id_(0)
{
    init_ui();
    init_event_bus();
}

// 1. 修改析构函数
VisualizationWindow::~VisualizationWindow()
{
    // Stop and delete update timer
    if (update_timer_) {
        update_timer_->stop();
        delete update_timer_;
        update_timer_ = nullptr;
    }
}

// 2. 修复closeEvent函数的作用域
void VisualizationWindow::closeEvent(QCloseEvent *event)
{
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

// 3. 实现init_ui()方法
void VisualizationWindow::init_ui()
{
    // Create central widget and main layout
    central_widget_ = new QWidget(this);
    main_layout_ = new QVBoxLayout(central_widget_);
    
    // Create communication mode selection layout and components
    communication_mode_layout_ = new QHBoxLayout();
    QLabel *mode_label = new QLabel("Communication Mode:", this);
    communication_mode_combo_ = new QComboBox(this);
    communication_mode_combo_->addItem("ROS", "ros");
    communication_mode_combo_->addItem("EventBus", "eventbus");
    communication_mode_layout_->addWidget(mode_label);
    communication_mode_layout_->addWidget(communication_mode_combo_);
    communication_mode_layout_->addStretch();
    
    // Create main data chart
    chart_ = new QChart();
    series_ = new QLineSeries();
    chart_->addSeries(series_);
    chart_view_ = new QChartView(chart_);
    chart_view_->setRenderHint(QPainter::Antialiasing);
    
    // Configure axes for main chart
    x_axis_ = new QValueAxis();
    y_axis_ = new QValueAxis();
    chart_->addAxis(x_axis_, Qt::AlignBottom);
    chart_->addAxis(y_axis_, Qt::AlignLeft);
    series_->attachAxis(x_axis_);
    series_->attachAxis(y_axis_);
    x_axis_->setTitleText("Time");
    y_axis_->setTitleText("Sensor Value");
    
    // Create latency chart
    latency_chart_ = new QChart();
    ros_latency_series_ = new QLineSeries();
    eventbus_latency_series_ = new QLineSeries();
    ros_average_latency_series_ = new QLineSeries();
    eventbus_average_latency_series_ = new QLineSeries();
    
    // Set series colors and names
    ros_latency_series_->setName("ROS Latency");
    ros_latency_series_->setColor(Qt::red);
    eventbus_latency_series_->setName("EventBus Latency");
    eventbus_latency_series_->setColor(Qt::blue);
    ros_average_latency_series_->setName("ROS Average Latency");
    ros_average_latency_series_->setColor(Qt::darkRed);
    ros_average_latency_series_->setPen(QPen(Qt::darkRed, 2, Qt::DotLine));
    eventbus_average_latency_series_->setName("EventBus Average Latency");
    eventbus_average_latency_series_->setColor(Qt::darkBlue);
    eventbus_average_latency_series_->setPen(QPen(Qt::darkBlue, 2, Qt::DotLine));
    
    // Configure axes for latency chart
    latency_x_axis_ = new QValueAxis();
    latency_y_axis_ = new QValueAxis();
    latency_chart_->addAxis(latency_x_axis_, Qt::AlignBottom);
    latency_chart_->addAxis(latency_y_axis_, Qt::AlignLeft);
    
    // Attach series to axes
    ros_latency_series_->attachAxis(latency_x_axis_);
    ros_latency_series_->attachAxis(latency_y_axis_);
    eventbus_latency_series_->attachAxis(latency_x_axis_);
    eventbus_latency_series_->attachAxis(latency_y_axis_);
    ros_average_latency_series_->attachAxis(latency_x_axis_);
    ros_average_latency_series_->attachAxis(latency_y_axis_);
    eventbus_average_latency_series_->attachAxis(latency_x_axis_);
    eventbus_average_latency_series_->attachAxis(latency_y_axis_);
    
    // Create latency chart view
    latency_chart_view_ = new QChartView(latency_chart_);
    latency_chart_view_->setRenderHint(QPainter::Antialiasing);
    latency_x_axis_->setTitleText("Time");
    latency_y_axis_->setTitleText("Latency (ms)");
    
    // Create latency labels layout
    latency_labels_layout_ = new QHBoxLayout();
    ros_average_latency_label_ = new QLabel("ROS Average Latency: 0.00 ms", this);
    eventbus_average_latency_label_ = new QLabel("EventBus Average Latency: 0.00 ms", this);
    eventbus_average_latency_label_->hide();  // Initially hide EventBus label
    latency_labels_layout_->addWidget(ros_average_latency_label_);
    latency_labels_layout_->addWidget(eventbus_average_latency_label_);
    latency_labels_layout_->addStretch();
    
    // Create zoom control layout
    zoom_layout_ = new QHBoxLayout();
    zoom_label_ = new QLabel("Zoom:", this);
    zoom_slider_ = new QSlider(Qt::Horizontal, this);
    zoom_slider_->setRange(50, 1000);
    zoom_slider_->setValue(max_data_points_);
    zoom_value_label_ = new QLabel(QString("%1 points").arg(max_data_points_), this);
    zoom_layout_->addWidget(zoom_label_);
    zoom_layout_->addWidget(zoom_slider_);
    zoom_layout_->addWidget(zoom_value_label_);
    zoom_layout_->addStretch();
    
    // Add all components to main layout
    main_layout_->addLayout(communication_mode_layout_);
    main_layout_->addWidget(chart_view_);
    main_layout_->addLayout(latency_labels_layout_);
    main_layout_->addWidget(latency_chart_view_);
    main_layout_->addLayout(zoom_layout_);
    
    // Set central widget
    setCentralWidget(central_widget_);
    
    // Create update timer
    update_timer_ = new QTimer(this);
    
    // Connect signals and slots
    connect(update_timer_, &QTimer::timeout, this, &VisualizationWindow::timerUpdate);
    connect(zoom_slider_, &QSlider::valueChanged, this, &VisualizationWindow::zoomChanged);
    connect(communication_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &VisualizationWindow::onCommunicationModeChanged);
    
    // Set initial zoom and start timer
    zoomChanged(max_data_points_);
    update_timer_->start(100);  // Update UI every 100 ms
    
    // Set window properties
    setWindowTitle("Sensor Data Visualization");
    resize(1000, 800);
}

// 4. 修改init_event_bus方法以保存subscription ID
void VisualizationWindow::init_event_bus()
{
    // 使用lambda表达式来调用成员函数并保存HandlerId
    eventbus_subscription_id_ = EVENT_BUS.subscribe("processed_sensor_data", 
        [this](const QVariantMap& event_data) { onEventBusDataReceived(event_data); });
}

// 5. 在类的命名空间内添加onRosDataReceived方法（确保它在命名空间内）
void VisualizationWindow::onEventBusDataReceived(const QVariantMap& event_data)
{
    // 只在选择EventBus通信方式时处理数据
    if (communication_mode_ == "eventbus") {
        // 从EventBus获取数据和时间戳
        double data = event_data["data"].toDouble();
        QDateTime timestamp = event_data["timestamp"].toDateTime();
        
        // 计算延迟
        QDateTime current_time = QDateTime::currentDateTime();
        double latency_ms = timestamp.msecsTo(current_time);
        
        // 输出EventBus延迟到终端
        std::cout << "EventBus延迟: " << latency_ms << " 毫秒" << std::endl;
        
        // 更新图表
        updateChart(data, latency_ms, false);  // false表示是EventBus数据
    }
}

// 在init_event_bus函数之后添加
void VisualizationWindow::onRosDataReceived(double value, double latency)
{
    // 只在选择ROS通信方式时处理数据
    if (communication_mode_ == "ros") {
        // 输出ROS延迟到终端
        std::cout << "ROS延迟: " << latency << " 毫秒" << std::endl;
        
        // 更新图表
        updateChart(value, latency, true);  // true表示是ROS数据
    }
}

void VisualizationWindow::updateChart(double value, double latency, bool is_ros)
{
    // 添加主数据点
    all_data_points_.append(QPointF(x_counter_, value));
    
    // 添加延迟数据点
    if (is_ros) {
        all_ros_latency_points_.append(QPointF(x_counter_, latency));
        ros_total_latency_ += latency;
        ros_latency_point_count_++;
    } else {
        all_eventbus_latency_points_.append(QPointF(x_counter_, latency));
        eventbus_total_latency_ += latency;
        eventbus_latency_point_count_++;
    }
    
    x_counter_ += 1.0;

    // 控制内存使用
    if (static_cast<size_t>(all_data_points_.size()) > max_storage_points_) {
        all_data_points_.removeFirst();
    }
    
    if (static_cast<size_t>(all_ros_latency_points_.size()) > max_storage_points_) {
        ros_total_latency_ -= all_ros_latency_points_.first().y();
        all_ros_latency_points_.removeFirst();
        ros_latency_point_count_--;
    }
    
    if (static_cast<size_t>(all_eventbus_latency_points_.size()) > max_storage_points_) {
        eventbus_total_latency_ -= all_eventbus_latency_points_.first().y();
        all_eventbus_latency_points_.removeFirst();
        eventbus_latency_point_count_--;
    }
}

void VisualizationWindow::timerUpdate()
{
    // 根据当前通信模式计算平均延迟
    double ros_average_latency = 0.0;
    double eventbus_average_latency = 0.0;
    
    if (communication_mode_ == "ros") {
        ros_average_latency = (ros_latency_point_count_ > 0) ? (ros_total_latency_ / ros_latency_point_count_) : 0.0;
        ros_average_latency_label_->setText(QString("ROS Average Latency: %1 ms").arg(ros_average_latency, 0, 'f', 2));
    } else {
        eventbus_average_latency = (eventbus_latency_point_count_ > 0) ? (eventbus_total_latency_ / eventbus_latency_point_count_) : 0.0;
        eventbus_average_latency_label_->setText(QString("EventBus Average Latency: %1 ms").arg(eventbus_average_latency, 0, 'f', 2));
    }
    
    // 确定要显示的数据点范围
    QVector<QPointF> display_data_points;
    QVector<QPointF> display_ros_latency_points;
    QVector<QPointF> display_eventbus_latency_points;
    QVector<QPointF> display_ros_average_points;
    QVector<QPointF> display_eventbus_average_points;
    
    int total_data_points = all_data_points_.size();
    int start_index = std::max(0, total_data_points - static_cast<int>(max_data_points_));
    
    // 只显示所有主数据点（无论通信模式）
    for (int i = start_index; i < total_data_points; ++i) {
        display_data_points.append(all_data_points_[i]);
        
        // 根据当前通信模式只添加对应的数据点
        if (communication_mode_ == "ros" && i < all_ros_latency_points_.size()) {
            display_ros_latency_points.append(all_ros_latency_points_[i]);
            display_ros_average_points.append(QPointF(all_ros_latency_points_[i].x(), ros_average_latency));
        } else if (communication_mode_ == "eventbus" && i < all_eventbus_latency_points_.size()) {
            display_eventbus_latency_points.append(all_eventbus_latency_points_[i]);
            display_eventbus_average_points.append(QPointF(all_eventbus_latency_points_[i].x(), eventbus_average_latency));
        }
    }

    // 更新主数据曲线
    series_->replace(display_data_points);
    
    // 根据当前通信模式更新对应的延迟曲线
    if (communication_mode_ == "ros") {
        ros_latency_series_->replace(display_ros_latency_points);
        ros_average_latency_series_->replace(display_ros_average_points);
    } else {
        eventbus_latency_series_->replace(display_eventbus_latency_points);
        eventbus_average_latency_series_->replace(display_eventbus_average_points);
    }

    // 自动调整主数据图表的坐标轴
    if (display_data_points.size() > 0) {
        double y_min = display_data_points.first().y();
        double y_max = display_data_points.first().y();
        
        for (const auto &point : display_data_points) {
            y_min = std::min(y_min, point.y());
            y_max = std::max(y_max, point.y());
        }
        
        double y_padding = (y_max - y_min) * 0.1;
        if (y_padding == 0) {
            y_padding = 0.5;
        }
        
        y_axis_->setRange(y_min - y_padding, y_max + y_padding);
        
        // 调整X轴范围
        if (static_cast<size_t>(total_data_points) < max_data_points_) {
            x_axis_->setRange(0, max_data_points_);
            latency_x_axis_->setRange(0, max_data_points_);
        } else {
            double x_end = x_counter_;
            double x_start = x_end - max_data_points_;
            x_axis_->setRange(x_start, x_end);
            latency_x_axis_->setRange(x_start, x_end);
        }
    }
    
    // 自动调整延迟图表的Y轴，只考虑当前通信模式的数据
    double latency_min = 0.0;
    double latency_max = 10.0;
    
    if (communication_mode_ == "ros" && !display_ros_latency_points.empty()) {
        for (const auto &point : display_ros_latency_points) {
            latency_min = std::min(latency_min, point.y());
            latency_max = std::max(latency_max, point.y());
        }
        latency_min = std::min(latency_min, ros_average_latency);
        latency_max = std::max(latency_max, ros_average_latency);
    } else if (communication_mode_ == "eventbus" && !display_eventbus_latency_points.empty()) {
        for (const auto &point : display_eventbus_latency_points) {
            latency_min = std::min(latency_min, point.y());
            latency_max = std::max(latency_max, point.y());
        }
        latency_min = std::min(latency_min, eventbus_average_latency);
        latency_max = std::max(latency_max, eventbus_average_latency);
    }
    
    double latency_padding = (latency_max - latency_min) * 0.1;
    if (latency_padding == 0) {
        latency_padding = 5.0;
    }
    
    latency_y_axis_->setRange(latency_min - latency_padding, latency_max + latency_padding);
}

void VisualizationWindow::zoomChanged(int value)
{
    // Update max_data_points_ with slider value
    max_data_points_ = static_cast<size_t>(value);
    
    // Update slider label
    zoom_value_label_->setText(QString("%1 points").arg(value));
    
    // 重新更新图表
    timerUpdate();
}

void VisualizationWindow::onCommunicationModeChanged(int) {
    // Get selected communication mode
    communication_mode_ = communication_mode_combo_->currentData().toString().toStdString();
    
    // Remove only series that are currently in the chart
    if (latency_chart_->series().contains(ros_latency_series_)) {
        latency_chart_->removeSeries(ros_latency_series_);
    }
    if (latency_chart_->series().contains(eventbus_latency_series_)) {
        latency_chart_->removeSeries(eventbus_latency_series_);
    }
    if (latency_chart_->series().contains(ros_average_latency_series_)) {
        latency_chart_->removeSeries(ros_average_latency_series_);
    }
    if (latency_chart_->series().contains(eventbus_average_latency_series_)) {
        latency_chart_->removeSeries(eventbus_average_latency_series_);
    }
    
    // Hide all latency labels
    ros_average_latency_label_->hide();
    eventbus_average_latency_label_->hide();
    
    // Add only the selected communication mode's series
    if (communication_mode_ == "ros") {
        latency_chart_->addSeries(ros_latency_series_);
        latency_chart_->addSeries(ros_average_latency_series_);
        ros_average_latency_label_->show();
    } else {
        latency_chart_->addSeries(eventbus_latency_series_);
        latency_chart_->addSeries(eventbus_average_latency_series_);
        eventbus_average_latency_label_->show();
    }
    
    // Clear all data points to start fresh with new communication mode
    all_data_points_.clear();
    series_->clear();
    all_ros_latency_points_.clear();
    all_eventbus_latency_points_.clear();
    ros_latency_series_->clear();
    eventbus_latency_series_->clear();
    ros_average_latency_series_->clear();
    eventbus_average_latency_series_->clear();
    
    x_counter_ = 0.0;
    ros_total_latency_ = 0.0;
    eventbus_total_latency_ = 0.0;
    ros_latency_point_count_ = 0;
    eventbus_latency_point_count_ = 0;
    
    // Update chart immediately
    timerUpdate();
    
    // 使用标准C++输出替代ROS日志
    std::cout << "Switched communication mode to: " << communication_mode_ << std::endl;
}

}  // namespace qt_visualization

// Remove this include
// #include "rclcpp_components/register_node_macro.hpp"

// Remove this line at the end of the file
// 删除文件末尾的以下代码（如果存在）
// RCLCPP_COMPONENTS_REGISTER_NODE(qt_visualization::VisualizationWindow)