#ifndef DATA_PROCESSING__SOFTWARE_FIFO_HPP_
#define DATA_PROCESSING__SOFTWARE_FIFO_HPP_

#include <vector>
#include <mutex>
#include <condition_variable>
#include <stdexcept>

template <typename T>
class SoftwareFIFO
{
public:
    explicit SoftwareFIFO(size_t capacity);
    ~SoftwareFIFO() = default;

    // Push data into FIFO
    bool push(const T & data);
    bool push(T && data);

    // Pop data from FIFO
    bool pop(T & data);

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

#endif  // DATA_PROCESSING__SOFTWARE_FIFO_HPP_