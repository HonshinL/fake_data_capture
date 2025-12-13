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
      total_latency_(0.0),
      latency_point_count_(0),
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
    latency_series_ = new QLineSeries();
    average_latency_series_ = new QLineSeries();
    
    // Set series colors and names based on communication mode
    if (communication_mode_ == "ros") {
        latency_series_->setName("ROS Latency");
        latency_series_->setColor(Qt::red);
        average_latency_series_->setName("ROS Average Latency");
        average_latency_series_->setColor(Qt::darkRed);
        average_latency_series_->setPen(QPen(Qt::darkRed, 2, Qt::DotLine));
    } else {
        latency_series_->setName("EventBus Latency");
        latency_series_->setColor(Qt::blue);
        average_latency_series_->setName("EventBus Average Latency");
        average_latency_series_->setColor(Qt::darkBlue);
        average_latency_series_->setPen(QPen(Qt::darkBlue, 2, Qt::DotLine));
    }
    
    // 修复：先将系列添加到图表中（必须在附加轴之前）
    latency_chart_->addSeries(latency_series_);
    latency_chart_->addSeries(average_latency_series_);
    
    // Configure axes for latency chart
    latency_x_axis_ = new QValueAxis();
    latency_y_axis_ = new QValueAxis();
    latency_chart_->addAxis(latency_x_axis_, Qt::AlignBottom);
    latency_chart_->addAxis(latency_y_axis_, Qt::AlignLeft);
    
    // 启用延迟图表图例
    latency_chart_->legend()->show();
    
    // Attach series to axes（现在系列已经在图表中，可以附加轴了）
    latency_series_->attachAxis(latency_x_axis_);
    latency_series_->attachAxis(latency_y_axis_);
    average_latency_series_->attachAxis(latency_x_axis_);
    average_latency_series_->attachAxis(latency_y_axis_);
    
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
    connect(update_timer_, &QTimer::timeout, this, &VisualizationWindow::updateCharts);
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
        latency_series_->setVisible(true);
        average_latency_series_->setVisible(true);
        ros_average_latency_label_->show();
        eventbus_average_latency_label_->hide();
    } else {
        latency_series_->setVisible(true);
        average_latency_series_->setVisible(true);
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
        updateDataPoints(data, latency_ms, false);
    }
}

// 在init_event_bus函数之后添加
// 修改onRosDataReceived方法，在其中计算延迟
void VisualizationWindow::onRosDataReceived(double value, double timestamp_us)
{
    // 只在选择ROS通信方式时处理数据
    if (communication_mode_ == "ros") {
        // 获取当前时间的微秒级精度
        QDateTime current_time = QDateTime::currentDateTimeUtc();
        double current_us = current_time.toMSecsSinceEpoch() * 1000.0;
        
        // 计算延迟（毫秒）
        double latency_ms = (current_us - timestamp_us) / 1000.0;
        
        // 输出ROS延迟到终端
        std::cout << "ROS延迟: " << latency_ms << " 毫秒" << std::endl;
        
        // 更新数据点
        updateDataPoints(value, latency_ms, true);  // true表示是ROS数据
    }
}

// 修改updateChart方法，优化延迟数据点的处理逻辑
void VisualizationWindow::updateDataPoints(double value, double latency, bool is_ros) 
{ 
    // 标记is_ros参数为故意未使用，避免编译警告 
    (void)is_ros; 
     
    // 添加主数据点 
    data_points_.append(QPointF(x_counter_, value)); 
     
    // 添加延迟数据点 
    latency_points_.append(QPointF(x_counter_, latency));
    
    // // 添加调试信息：当数据点数量大于10时打印并退出
    // if (data_points_.size() > 10) {
    //     std::cout << "调试信息：" << std::endl;
    //     std::cout << "主数据点数量：" << data_points_.size() << std::endl;
    //     std::cout << "延迟数据点数量：" << latency_points_.size() << std::endl;
    //     std::cout << "最后10个主数据点：" << std::endl;
        
    //     // 打印最后10个主数据点
    //     int start = std::max(0, static_cast<int>(data_points_.size()) - 10);
    //     for (int i = start; i < data_points_.size(); ++i) {
    //         std::cout << "  索引" << i << ": x=" << data_points_[i].x() << ", y=" << data_points_[i].y() << std::endl;
    //     }
        
    //     std::cout << "最后10个延迟数据点：" << std::endl;
    //     // 打印最后10个延迟数据点
    //     start = std::max(0, static_cast<int>(latency_points_.size()) - 10);
    //     for (int i = start; i < latency_points_.size(); ++i) {
    //         std::cout << "  索引" << i << ": x=" << latency_points_[i].x() << ", y=" << latency_points_[i].y() << std::endl;
    //     }
        
    //     // 退出程序
    //     std::cout << "数据点数量超过10，程序退出。" << std::endl;
    //     QApplication::quit();
    // }
    
    // 修复：删除了额外的大括号，确保以下代码在函数体内
    total_latency_ += latency;
    latency_point_count_++;
    
    // 确保主数据点数量不超过最大存储量
    if (static_cast<size_t>(data_points_.size()) > max_storage_points_) {
        data_points_.removeFirst();
    }
    
    // 确保延迟数据点数量不超过最大存储量
    if (static_cast<size_t>(latency_points_.size()) > max_storage_points_) {
        total_latency_ -= latency_points_.first().y();
        latency_points_.removeFirst();
        latency_point_count_--;
    }
    
    x_counter_ += 1.0;
}

// 修改updateCharts方法，优化延迟数据点的显示逻辑
// 添加一个通用的图表更新辅助方法
void VisualizationWindow::updateChartSeries(QLineSeries* series, QValueAxis* y_axis, 
                                           const QVector<QPointF>& data_points, 
                                           double padding_factor, double min_padding)
{
    // 更新曲线数据
    series->replace(data_points);
    
    // 自动调整Y轴范围
    if (data_points.size() > 0) {
        double y_min = data_points.first().y();
        double y_max = data_points.first().y();
        
        for (const auto &point : data_points) {
            y_min = std::min(y_min, point.y());
            y_max = std::max(y_max, point.y());
        }
        
        double y_padding = (y_max - y_min) * padding_factor;
        if (y_padding < min_padding) {
            y_padding = min_padding;
        }
        
        y_axis->setRange(y_min - y_padding, y_max + y_padding);
    } else {
        // 没有数据时，设置一个默认的合理范围
        y_axis->setRange(-1.0, 1.0);
    }
}

// 更新updateCharts方法，使用统一的更新逻辑
void VisualizationWindow::updateCharts()
{
    // 根据当前通信模式计算平均延迟
    double average_latency = (latency_point_count_ > 0) ? (total_latency_ / latency_point_count_) : 0.0;
    
    if (communication_mode_ == "ros") {
        ros_average_latency_label_->setText(QString("ROS Average Latency: %1 ms").arg(average_latency, 0, 'f', 2));
    } else {
        eventbus_average_latency_label_->setText(QString("EventBus Average Latency: %1 ms").arg(average_latency, 0, 'f', 2));
    }
    
    // 确定要显示的数据点范围
    QVector<QPointF> display_data_points;
    QVector<QPointF> display_latency_points;
    QVector<QPointF> display_average_points;
    
    int total_data_points = data_points_.size();
    int start_index = std::max(0, total_data_points - static_cast<int>(max_data_points_));
    
    // 只显示所有主数据点（无论通信模式）
    for (int i = start_index; i < total_data_points; ++i) {
        display_data_points.append(data_points_[i]);
    }
    
    // 延迟图表使用与主数据图表完全相同的索引范围
    int total_latency_points = latency_points_.size();
    
    // 修改后
    int latency_start_index = std::max(0, total_latency_points - static_cast<int>(max_data_points_));
    int latency_end_index = total_latency_points;
    
    // 确保索引在有效范围内
    if (latency_start_index < 0) latency_start_index = 0;
    if (latency_end_index > total_latency_points) {
        latency_end_index = total_latency_points;
    }
    
    // 只显示所有延迟数据点（使用与主数据图表相同的索引范围）
    for (int i = latency_start_index; i < latency_end_index; ++i) {
        display_latency_points.append(latency_points_[i]);
        display_average_points.append(QPointF(latency_points_[i].x(), average_latency));
    }
    
    // 调整X轴范围，确保主图表和延迟图表完全同步
    if (display_data_points.size() > 0) {
        double x_end = x_counter_;
        double x_start = std::max(0.0, x_end - max_data_points_);
        x_axis_->setRange(x_start, x_end);
        latency_x_axis_->setRange(x_start, x_end);
    }
    
    // 使用统一的方法更新主数据图表
    updateChartSeries(series_, y_axis_, display_data_points, 0.1, 0.5);
    
    // 使用统一的方法更新延迟图表
    updateChartSeries(latency_series_, latency_y_axis_, display_latency_points, 0.1, 1.0);
    
    // 更新平均延迟曲线
    // 移除重复的latency_series_->replace(display_latency_points);调用
    average_latency_series_->replace(display_average_points);
    
    // 如果有延迟数据，调整Y轴范围时需要考虑平均延迟线
    if (display_latency_points.size() > 0) {
        double latency_min = latency_y_axis_->min();
        double latency_max = latency_y_axis_->max();
        
        // 确保平均延迟线也在可见范围内
        latency_min = std::min(latency_min, average_latency);
        latency_max = std::max(latency_max, average_latency);
        
        double latency_padding = (latency_max - latency_min) * 0.1;
        if (latency_padding < 1.0) { // 确保至少有1.0的padding
            latency_padding = 1.0;
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
    
    // 重新更新图表
    updateCharts();
}

void VisualizationWindow::onCommunicationModeChanged(int) {
    // Get selected communication mode
    communication_mode_ = communication_mode_combo_->currentData().toString().toStdString();
    
    // Hide all latency labels
    ros_average_latency_label_->hide();
    eventbus_average_latency_label_->hide();
    
    // Update series name and color based on selected communication mode
    if (communication_mode_ == "ros") {
        latency_series_->setName("ROS Latency");
        latency_series_->setColor(Qt::red);
        average_latency_series_->setName("ROS Average Latency");
        average_latency_series_->setColor(Qt::darkRed);
        average_latency_series_->setPen(QPen(Qt::darkRed, 2, Qt::DotLine));
        ros_average_latency_label_->show();
    } else {
        latency_series_->setName("EventBus Latency");
        latency_series_->setColor(Qt::blue);
        average_latency_series_->setName("EventBus Average Latency");
        average_latency_series_->setColor(Qt::darkBlue);
        average_latency_series_->setPen(QPen(Qt::darkBlue, 2, Qt::DotLine));
        eventbus_average_latency_label_->show();
    }
    
    // Clear all data points to start fresh with new communication mode
    data_points_.clear();
    series_->clear();
    latency_points_.clear();
    latency_series_->clear();
    average_latency_series_->clear();
    
    x_counter_ = 0.0;
    total_latency_ = 0.0;
    latency_point_count_ = 0;
    
    // Update chart immediately
    updateCharts();
    
    // 使用标准C++输出替代ROS日志
    std::cout << "Switched communication mode to: " << communication_mode_ << std::endl;
}
}  // namespace qt_visualization