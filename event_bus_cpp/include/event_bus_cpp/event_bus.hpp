// event_bus/event_bus.hpp
#ifndef EVENT_BUS_HPP
#define EVENT_BUS_HPP

#include <QObject>
#include <QString>
#include <QMap>
#include <QList>
#include <QMutex>
#include <functional>
#include <memory>
#include <QVariantMap> // 添加QVariantMap的包含

class EventBus : public QObject
{
    Q_OBJECT

public:
    using EventHandler = std::function<void(const QVariantMap&)>;
    using HandlerId = size_t;
    
    static EventBus& instance();
    
    // 订阅事件，返回处理函数ID用于取消订阅
    HandlerId subscribe(const QString& eventName, const EventHandler& handler);
    
    // 取消订阅
    void unsubscribe(const QString& eventName, HandlerId handlerId);
    
    // 发布事件
    void publish(const QString& eventName, const QVariantMap& params = QVariantMap());
    
    // 获取订阅者数量（用于调试）
    int getSubscriberCount(const QString& eventName) const;
    
    // 列出所有事件（用于调试）
    QStringList listEvents() const;

private:
    // 处理函数包装器，包含ID和函数
    struct HandlerWrapper {
        HandlerId id;
        EventHandler handler;
    };

signals:
    // 内部信号，用于线程安全的事件处理
    void eventTriggered(const QString& eventName, const QVariantMap& params);

private slots:
    // 槽函数，处理事件
    void handleEvent(const QString& eventName, const QVariantMap& params);

private:
    EventBus(QObject* parent = nullptr);
    
    QMap<QString, QList<HandlerWrapper>> m_handlers;
    mutable QMutex m_mutex;
    HandlerId m_nextHandlerId = 0; // 用于生成唯一ID
};

// 全局事件总线实例宏
#define EVENT_BUS EventBus::instance()

#endif // EVENT_BUS_HPP