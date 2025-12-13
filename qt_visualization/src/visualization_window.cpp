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
    series_->setName("Sensor Value");  // 添加这行代码为主要数据系列设置名称
    chart_->addSeries(series_);
    chart_view_ = new QChartView(chart_);
    chart_view_->setRenderHint(QPainter::Antialiasing);
    // 启用图例
    chart_->legend()->show();
    
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
    
    // 启用延迟图表图例
    latency_chart_->legend()->show();  // 添加这行代码
    
    // Attach series to axes
    ros_latency_series_->attachAxis(latency_x_axis_);
    ros_latency_series_->attachAxis(latency_y_axis_);
    eventbus_latency_series_->attachAxis(latency_x_axis_);
    eventbus_latency_series_->attachAxis(latency_y_axis_);
    ros_average_latency_series_->attachAxis(latency_x_axis_);
    ros_average_latency_series_->attachAxis(latency_y_axis_);
    eventbus_average_latency_series_->attachAxis(latency_x_axis_);
    eventbus_average_latency_series_->attachAxis(latency_y_axis_);
    
    // 添加这部分代码，将延迟系列添加到延迟图表中
    latency_chart_->addSeries(ros_latency_series_);
    latency_chart_->addSeries(eventbus_latency_series_);
    latency_chart_->addSeries(ros_average_latency_series_);
    latency_chart_->addSeries(eventbus_average_latency_series_);
    
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
    latency_labels_layout_->addStretch();  // 添加伸缩项使ROS标签居中
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
    zoom_layout_->addWidget(zoom_slider_, 1);  // 设置拉伸因子为1，让滑块占据剩余空间
    zoom_layout_->addWidget(zoom_value_label_);
    // 移除末尾的addStretch()，这样整个zoom栏会充满可用宽度
    zoom_layout_->addStretch();
    
    // Add all components to main layout - 调整顺序，将标签放在图表下方
    main_layout_->addLayout(communication_mode_layout_);
    main_layout_->addWidget(chart_view_);
    main_layout_->addWidget(latency_chart_view_);  // 先添加图表
    main_layout_->addLayout(latency_labels_layout_);  // 再添加标签
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
    
    // 初始化时根据默认通信模式设置系列可见性
    if (communication_mode_ == "ros") {
        ros_latency_series_->setVisible(true);
        ros_average_latency_series_->setVisible(true);
        eventbus_latency_series_->setVisible(false);
        eventbus_average_latency_series_->setVisible(false);
        ros_average_latency_label_->show();
        eventbus_average_latency_label_->hide();
    } else {
        ros_latency_series_->setVisible(false);
        ros_average_latency_series_->setVisible(false);
        eventbus_latency_series_->setVisible(true);
        eventbus_average_latency_series_->setVisible(true);
        ros_average_latency_label_->hide();
        eventbus_average_latency_label_->show();
    }
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
    if (communication_mode_ == "eventbus") {
        double data = event_data["data"].toDouble();
        double timestamp_us = event_data["timestamp_us"].toDouble();
        
        // 获取当前时间的微秒级精度
        QDateTime current_time = QDateTime::currentDateTimeUtc();
        double current_us = current_time.toMSecsSinceEpoch() * 1000.0;
        double latency_ms = (current_us - timestamp_us) / 1000.0;
        
        std::cout << "EventBus延迟: " << latency_ms << " 毫秒" << std::endl;
        updateChart(data, latency_ms, false);
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

// 修改updateChart方法，优化延迟数据点的处理逻辑
void VisualizationWindow::updateChart(double value, double latency, bool is_ros)
{
    // 添加主数据点
    all_data_points_.append(QPointF(x_counter_, value));
    
    // 添加延迟数据点
    if (is_ros) {
        all_ros_latency_points_.append(QPointF(x_counter_, latency));
        ros_total_latency_ += latency;
        ros_latency_point_count_++;
        
        // 确保ROS延迟数据点数量不超过最大存储量
        if (static_cast<size_t>(all_ros_latency_points_.size()) > max_storage_points_) {
            ros_total_latency_ -= all_ros_latency_points_.first().y();
            all_ros_latency_points_.removeFirst();
            ros_latency_point_count_--;
        }
    } else {
        all_eventbus_latency_points_.append(QPointF(x_counter_, latency));
        eventbus_total_latency_ += latency;
        eventbus_latency_point_count_++;
        
        // 确保EventBus延迟数据点数量不超过最大存储量
        if (static_cast<size_t>(all_eventbus_latency_points_.size()) > max_storage_points_) {
            eventbus_total_latency_ -= all_eventbus_latency_points_.first().y();
            all_eventbus_latency_points_.removeFirst();
            eventbus_latency_point_count_--;
        }
    }
    
    x_counter_ += 1.0;

    // 确保主数据点数量不超过最大存储量
    if (static_cast<size_t>(all_data_points_.size()) > max_storage_points_) {
        all_data_points_.removeFirst();
    }
}

// 修改timerUpdate方法，优化延迟数据点的显示逻辑
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
    }
    
    // 分别处理延迟数据点，确保与主数据点完全同步
    if (communication_mode_ == "ros") {
        // 使用与主数据点相同的索引范围
        int ros_start_index = start_index;
        int ros_end_index = total_data_points;
        
        // 确保索引在有效范围内
        if (ros_start_index < 0) ros_start_index = 0;
        if (ros_end_index > static_cast<int>(all_ros_latency_points_.size())) {
            ros_end_index = static_cast<int>(all_ros_latency_points_.size());
        }
        
        // 如果延迟数据点数量少于主数据点，调整起始索引
        if (ros_end_index - ros_start_index < static_cast<int>(max_data_points_)) {
            ros_start_index = std::max(0, ros_end_index - static_cast<int>(max_data_points_));
        }
        
        // 确保起始索引不大于结束索引
        if (ros_start_index >= ros_end_index) {
            ros_start_index = 0;
        }
        
        for (int i = ros_start_index; i < ros_end_index; ++i) {
            display_ros_latency_points.append(all_ros_latency_points_[i]);
            display_ros_average_points.append(QPointF(all_ros_latency_points_[i].x(), ros_average_latency));
        }
    } else if (communication_mode_ == "eventbus") {
        // 使用与主数据点相同的索引范围
        int eventbus_start_index = start_index;
        int eventbus_end_index = total_data_points;
        
        // 确保索引在有效范围内
        if (eventbus_start_index < 0) eventbus_start_index = 0;
        if (eventbus_end_index > static_cast<int>(all_eventbus_latency_points_.size())) {
            eventbus_end_index = static_cast<int>(all_eventbus_latency_points_.size());
        }
        
        // 如果延迟数据点数量少于主数据点，调整起始索引
        if (eventbus_end_index - eventbus_start_index < static_cast<int>(max_data_points_)) {
            eventbus_start_index = std::max(0, eventbus_end_index - static_cast<int>(max_data_points_));
        }
        
        // 确保起始索引不大于结束索引
        if (eventbus_start_index >= eventbus_end_index) {
            eventbus_start_index = 0;
        }
        
        for (int i = eventbus_start_index; i < eventbus_end_index; ++i) {
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
        
        // 调整X轴范围，确保主图表和延迟图表完全同步
        double x_end = x_counter_;
        double x_start = std::max(0.0, x_end - max_data_points_);
        x_axis_->setRange(x_start, x_end);
        latency_x_axis_->setRange(x_start, x_end);
    }
    
    // 自动调整延迟图表的Y轴，只考虑当前通信模式的数据
    double latency_min = 0.0;
    double latency_max = 0.0;
    bool has_data = false;
    
    if (communication_mode_ == "ros" && !display_ros_latency_points.empty()) {
        latency_min = display_ros_latency_points.first().y();
        latency_max = display_ros_latency_points.first().y();
        has_data = true;
        
        for (const auto &point : display_ros_latency_points) {
            latency_min = std::min(latency_min, point.y());
            latency_max = std::max(latency_max, point.y());
        }
        latency_min = std::min(latency_min, ros_average_latency);
        latency_max = std::max(latency_max, ros_average_latency);
    } else if (communication_mode_ == "eventbus" && !display_eventbus_latency_points.empty()) {
        latency_min = display_eventbus_latency_points.first().y();
        latency_max = display_eventbus_latency_points.first().y();
        has_data = true;
        
        for (const auto &point : display_eventbus_latency_points) {
            latency_min = std::min(latency_min, point.y());
            latency_max = std::max(latency_max, point.y());
        }
        latency_min = std::min(latency_min, eventbus_average_latency);
        latency_max = std::max(latency_max, eventbus_average_latency);
    }
    
    double latency_padding = 1.0; // 默认padding
    if (has_data) {
        latency_padding = (latency_max - latency_min) * 0.1;
        if (latency_padding < 1.0) { // 确保至少有1.0的padding
            latency_padding = 1.0;
        }
    } else {
        // 没有数据时，设置一个默认的合理范围
        latency_min = -1.0;
        latency_max = 1.0;
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
    
    // Hide all latency labels
    ros_average_latency_label_->hide();
    eventbus_average_latency_label_->hide();
    
    // Set series visibility based on selected communication mode
    if (communication_mode_ == "ros") {
        ros_latency_series_->setVisible(true);
        ros_average_latency_series_->setVisible(true);
        eventbus_latency_series_->setVisible(false);
        eventbus_average_latency_series_->setVisible(false);
        ros_average_latency_label_->show();
    } else {
        ros_latency_series_->setVisible(false);
        ros_average_latency_series_->setVisible(false);
        eventbus_latency_series_->setVisible(true);
        eventbus_average_latency_series_->setVisible(true);
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