#pragma once

#include <cstddef>
#include <cstdint>

namespace tools {

class RingBuffer {
public:
    RingBuffer() = default;
    RingBuffer(uint8_t* buffer, std::size_t capacity);

    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;

    bool set_buffer(uint8_t* buffer, std::size_t capacity);
    void reset();

    bool valid() const { return buffer_ != nullptr && capacity_ > 0U; }
    std::size_t capacity() const { return capacity_; }
    std::size_t size() const { return size_; }
    std::size_t available() const { return capacity_ - size_; }
    bool empty() const { return size_ == 0U; }
    bool full() const { return size_ == capacity_; }

    bool push(uint8_t value);
    bool push_overwrite(uint8_t value);
    std::size_t write(const uint8_t* data, std::size_t count);
    std::size_t write_overwrite(const uint8_t* data, std::size_t count);

    bool pop(uint8_t& value);
    std::size_t read(uint8_t* data, std::size_t count);

    bool peek(std::size_t offset, uint8_t& value) const;
    std::size_t discard(std::size_t count);

private:
    std::size_t next_index(std::size_t index) const;

    uint8_t* buffer_ = nullptr;
    std::size_t capacity_ = 0U;
    std::size_t head_ = 0U;
    std::size_t tail_ = 0U;
    std::size_t size_ = 0U;
};

} // namespace tools

namespace BSP {
using RingBuffer = tools::RingBuffer;
} // namespace BSP
