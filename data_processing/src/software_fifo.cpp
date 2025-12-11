#include "data_processing/software_fifo.hpp"

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

// Explicit template instantiation for double type
template class SoftwareFIFO<double>;