// circular_buffer.h - Implementation of a Circular Buffer Class
// Reference: https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc



#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <memory>

//#include <mutex>
/**
* Important Usage Note: This library reserves one spare entry for queue-full detection
* Otherwise, corner cases and detecting difference between full/empty is hard.
* You are not seeing an accidental off-by-one.
*/

template <class T>
class circular_buffer {
public:
    circular_buffer(size_t size) :
        buf_(std::unique_ptr<T[]>(new T[size])),
        size_(size)
    {
        //empty constructor
    }

    void put(T item)
    {
        //std::lock_guard<std::mutex> lock(mutex_);

        buf_[head_] = item;
        head_ = (head_ + 1) % size_;

        if(head_ == tail_)
        {
            tail_ = (tail_ + 1) % size_;
        }
    }

    T get(void)
    {
        //std::lock_guard<std::mutex> lock(mutex_);

        if(empty())
        {
            return T();
        }

        //Read data and advance the tail (we now have a free space)
        auto val = buf_[tail_];
        tail_ = (tail_ + 1) % size_;

        return val;
    }

    void reset(void)
    {
        //std::lock_guard<std::mutex> lock(mutex_);
        head_ = tail_;
    }

    bool empty(void)
    {
        //if head and tail are equal, we are empty
        return head_ == tail_;
    }

    bool full(void)
    {
        //If tail is ahead the head by 1, we are full
        return ((head_ + 1) % size_) == tail_;
    }

    size_t size(void)
    {
        return size_ - 1;
    }

private:
    //std::mutex mutex_;
    std::unique_ptr<T[]> buf_;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t size_;
};

typedef circular_buffer<uint8_t> cBuffer;
#endif // CIRCULAR_BUFFER_H
