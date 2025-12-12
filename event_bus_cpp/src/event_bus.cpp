// event_bus/event_bus.cpp
#include "event_bus_cpp/event_bus.hpp"
#include <QDebug>

EventBus::EventBus(QObject* parent) : QObject(parent)
{
    connect(this, &EventBus::eventTriggered, this, &EventBus::handleEvent, Qt::QueuedConnection);
}

EventBus& EventBus::instance()
{
    static EventBus instance;
    return instance;
}

EventBus::HandlerId EventBus::subscribe(const QString& eventName, const EventHandler& handler)
{
    QMutexLocker locker(&m_mutex);
    
    if (!m_handlers.contains(eventName)) {
        m_handlers[eventName] = QList<HandlerWrapper>();
    }
    
    // 创建新的处理函数包装器并分配唯一ID
    HandlerWrapper wrapper;
    wrapper.id = m_nextHandlerId++;
    wrapper.handler = handler;
    
    m_handlers[eventName].append(wrapper);
    return wrapper.id; // 返回ID供取消订阅使用
}

void EventBus::unsubscribe(const QString& eventName, HandlerId handlerId)
{
    QMutexLocker locker(&m_mutex);
    
    if (m_handlers.contains(eventName)) {
        auto& handlers = m_handlers[eventName];
        // 查找并移除指定ID的处理函数
        for (int i = 0; i < handlers.size(); ++i) {
            if (handlers[i].id == handlerId) {
                handlers.removeAt(i);
                break;
            }
        }
        
        if (handlers.isEmpty()) {
            m_handlers.remove(eventName);
        }
    }
}

void EventBus::publish(const QString& eventName, const QVariantMap& params)
{
    emit eventTriggered(eventName, params);
}

void EventBus::handleEvent(const QString& eventName, const QVariantMap& params)
{
    QList<HandlerWrapper> handlers;
    
    {   
        QMutexLocker locker(&m_mutex);
        if (m_handlers.contains(eventName)) {
            handlers = m_handlers[eventName];
        }
    }
    
    for (const auto& wrapper : handlers) {
        try {
            wrapper.handler(params);
        } catch (const std::exception& e) {
            qWarning() << "Error in event handler for" << eventName << ":" << e.what();
        }
    }
}

int EventBus::getSubscriberCount(const QString& eventName) const
{
    QMutexLocker locker(&m_mutex);
    return m_handlers.value(eventName).size();
}

QStringList EventBus::listEvents() const
{
    QMutexLocker locker(&m_mutex);
    return m_handlers.keys();
}