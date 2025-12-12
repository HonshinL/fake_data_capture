#ifndef DATA_PROCESSING__SOFTWARE_FIFO_HPP_
#define DATA_PROCESSING__SOFTWARE_FIFO_HPP_

#include <vector>
#include <mutex>
#include <condition_variable>
#include <stdexcept>
#include <chrono>

template <typename T>
class SoftwareFIFO
{
public:
    explicit SoftwareFIFO(size_t capacity);
    ~SoftwareFIFO() = default;

    // Push data into FIFO
    bool push(const T & data);
    bool push(T && data);

    // Pop data from FIFO (blocking and non-blocking versions)
    bool pop(T & data);
    bool pop_blocking(T & data, const std::chrono::milliseconds& timeout = std::chrono::milliseconds(100));

    // Get FIFO status
    bool is_empty() const;
    bool is_full() const;
    size_t size() const;
    size_t capacity() const;

    // Clear FIFO
    void clear();

private:
    std::vector<T> buffer_;
    size_t capacity_;
    size_t head_;
    size_t tail_;
    size_t size_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
};

// Template implementation
template <typename T>
SoftwareFIFO<T>::SoftwareFIFO(size_t capacity)
    : capacity_(capacity),
      head_(0),
      tail_(0),
      size_(0)
{
    if (capacity_ == 0) {
        throw std::invalid_argument("FIFO capacity must be greater than 0");
    }
    buffer_.resize(capacity_);
}

template <typename T>
bool SoftwareFIFO<T>::push(const T & data)
{
    std::unique_lock<std::mutex> lock(mutex_);

    if (is_full()) {
        // FIFO is full, overwrite oldest data (circular buffer behavior)
        buffer_[tail_] = data;
        tail_ = (tail_ + 1) % capacity_;
        head_ = (head_ + 1) % capacity_;  // Move head forward to maintain size
        not_empty_.notify_one();
        return false;  // Indicate that data was overwritten
    }

    buffer_[tail_] = data;
    tail_ = (tail_ + 1) % capacity_;
    ++size_;
    not_empty_.notify_one();
    return true;
}

template <typename T>
bool SoftwareFIFO<T>::push(T && data)
{
    std::unique_lock<std::mutex> lock(mutex_);

    if (is_full()) {
        // FIFO is full, overwrite oldest data
        buffer_[tail_] = std::move(data);
        tail_ = (tail_ + 1) % capacity_;
        head_ = (head_ + 1) % capacity_;
        not_empty_.notify_one();
        return false;
    }

    buffer_[tail_] = std::move(data);
    tail_ = (tail_ + 1) % capacity_;
    ++size_;
    not_empty_.notify_one();
    return true;
}

template <typename T>
bool SoftwareFIFO<T>::pop(T & data)
{
    std::unique_lock<std::mutex> lock(mutex_);

    if (is_empty()) {
        return false;
    }

    data = buffer_[head_];
    head_ = (head_ + 1) % capacity_;
    --size_;
    not_full_.notify_one();
    return true;
}

template <typename T>
bool SoftwareFIFO<T>::pop_blocking(T & data, const std::chrono::milliseconds& timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待直到有数据或超时
    if (!not_empty_.wait_for(lock, timeout, [this] { return !is_empty(); })) {
        return false;  // 超时
    }

    if (is_empty()) {
        return false;
    }

    data = buffer_[head_];
    head_ = (head_ + 1) % capacity_;
    --size_;
    not_full_.notify_one();
    return true;
}

template <typename T>
bool SoftwareFIFO<T>::is_empty() const
{
    return size_ == 0;
}

template <typename T>
bool SoftwareFIFO<T>::is_full() const
{
    return size_ == capacity_;
}

template <typename T>
size_t SoftwareFIFO<T>::size() const
{
    return size_;
}

template <typename T>
size_t SoftwareFIFO<T>::capacity() const
{
    return capacity_;
}

template <typename T>
void SoftwareFIFO<T>::clear()
{
    std::unique_lock<std::mutex> lock(mutex_);
    head_ = tail_ = size_ = 0;
    not_full_.notify_all();
}

#endif  // DATA_PROCESSING__SOFTWARE_FIFO_HPP_