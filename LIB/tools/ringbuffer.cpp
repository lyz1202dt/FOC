#include "ringbuffer.hpp"

namespace tools {

RingBuffer::RingBuffer(uint8_t* buffer, std::size_t capacity)
{
    (void)set_buffer(buffer, capacity);
}

bool RingBuffer::set_buffer(uint8_t* buffer, std::size_t capacity)
{
    if (buffer == nullptr || capacity == 0U) {
        buffer_ = nullptr;
        capacity_ = 0U;
        reset();
        return false;
    }

    buffer_ = buffer;
    capacity_ = capacity;
    reset();
    return true;
}

void RingBuffer::reset()
{
    head_ = 0U;
    tail_ = 0U;
    size_ = 0U;
}

bool RingBuffer::push(uint8_t value)
{
    if (!valid() || full()) {
        return false;
    }

    buffer_[head_] = value;
    head_ = next_index(head_);
    ++size_;
    return true;
}

bool RingBuffer::push_overwrite(uint8_t value)
{
    if (!valid()) {
        return false;
    }

    buffer_[head_] = value;
    head_ = next_index(head_);
    if (full()) {
        tail_ = next_index(tail_);
    } else {
        ++size_;
    }
    return true;
}

std::size_t RingBuffer::write(const uint8_t* data, std::size_t count)
{
    if (data == nullptr) {
        return 0U;
    }

    std::size_t written = 0U;
    while (written < count && push(data[written])) {
        ++written;
    }
    return written;
}

std::size_t RingBuffer::write_overwrite(const uint8_t* data, std::size_t count)
{
    if (data == nullptr || !valid()) {
        return 0U;
    }

    for (std::size_t i = 0U; i < count; ++i) {
        (void)push_overwrite(data[i]);
    }
    return count;
}

bool RingBuffer::pop(uint8_t& value)
{
    if (empty()) {
        return false;
    }

    value = buffer_[tail_];
    tail_ = next_index(tail_);
    --size_;
    return true;
}

std::size_t RingBuffer::read(uint8_t* data, std::size_t count)
{
    if (data == nullptr) {
        return 0U;
    }

    std::size_t read_count = 0U;
    while (read_count < count && pop(data[read_count])) {
        ++read_count;
    }
    return read_count;
}

bool RingBuffer::peek(std::size_t offset, uint8_t& value) const
{
    if (!valid() || offset >= size_) {
        return false;
    }

    value = buffer_[(tail_ + offset) % capacity_];
    return true;
}

std::size_t RingBuffer::discard(std::size_t count)
{
    if (!valid() || count == 0U || size_ == 0U) {
        return 0U;
    }

    if (count > size_) {
        count = size_;
    }

    tail_ = (tail_ + count) % capacity_;
    size_ -= count;
    return count;
}

std::size_t RingBuffer::next_index(std::size_t index) const
{
    return (index + 1U) % capacity_;
}

} // namespace tools
