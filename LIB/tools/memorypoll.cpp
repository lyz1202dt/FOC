#include "memorypoll.hpp"

#include "stm32g4xx.h"

#include <cstdlib>
#include <cstring>
#include <limits>

namespace tools {

namespace {

class InterruptGuard {
public:
    InterruptGuard() : primask_(__get_PRIMASK())
    {
        __disable_irq();
        __DMB();
    }

    ~InterruptGuard()
    {
        __DMB();
        __set_PRIMASK(primask_);
    }

    InterruptGuard(const InterruptGuard&) = delete;
    InterruptGuard& operator=(const InterruptGuard&) = delete;

private:
    uint32_t primask_;
};

}

MemoryPoll::MemoryPoll(std::size_t blob_size, std::size_t blob_count)
{
    if (blob_size == 0U || blob_count == 0U) {
        return;
    }

    constexpr std::size_t kAlign = alignof(std::max_align_t);
    constexpr std::size_t kMaxSize = std::numeric_limits<std::size_t>::max();
    if (blob_size < sizeof(void*)) {
        blob_size = sizeof(void*);
    }
    if (blob_size > kMaxSize - (kAlign - 1U)) {
        return;
    }
    const std::size_t aligned_size = (blob_size + kAlign - 1U) & ~(kAlign - 1U);
    if (blob_count > kMaxSize / aligned_size) {
        return;
    }
    const std::size_t storage_size = blob_count * aligned_size;
    const std::size_t bitmap_size = blob_count / 8U + (blob_count % 8U != 0U);
    if (storage_size > kMaxSize - bitmap_size) {
        return;
    }

    storage_ = static_cast<uint8_t*>(std::malloc(storage_size + bitmap_size));
    if (storage_ == nullptr) {
        return;
    }
    blob_size_ = aligned_size;
    blob_count_ = blob_count;
    allocated_ = storage_ + storage_size;
    std::memset(allocated_, 0, bitmap_size);

    uint8_t* cursor = storage_;
    for (std::size_t i = 0U; i < blob_count_; ++i) {
        void* blob = cursor;
        std::memcpy(blob, &free_head_, sizeof(free_head_));
        free_head_ = blob;
        cursor += blob_size_;
    }
}

MemoryPoll::~MemoryPoll()
{
    std::free(storage_);
    storage_ = nullptr;
    allocated_ = nullptr;
    free_head_ = nullptr;
    blob_size_ = 0U;
    blob_count_ = 0U;
}

void* MemoryPoll::require_blob()
{
    const InterruptGuard guard;
    if (free_head_ == nullptr) {
        return nullptr;
    }

    void* blob = free_head_;
    std::memcpy(&free_head_, blob, sizeof(free_head_));
    const std::size_t index = (static_cast<uint8_t*>(blob) - storage_) / blob_size_;
    allocated_[index / 8U] |= static_cast<uint8_t>(1U << (index % 8U));
    return blob;
}

bool MemoryPoll::release_blob(void* blob)
{
    if (blob == nullptr || storage_ == nullptr) {
        return false;
    }

    const uintptr_t begin = reinterpret_cast<uintptr_t>(storage_);
    const uintptr_t addr = reinterpret_cast<uintptr_t>(blob);

    if (addr < begin) {
        return false;
    }
    const uintptr_t offset = addr - begin;
    if (offset >= blob_count_ * blob_size_ || offset % blob_size_ != 0U) {
        return false;
    }
    const std::size_t index = offset / blob_size_;
    const uint8_t mask = static_cast<uint8_t>(1U << (index % 8U));

    const InterruptGuard guard;
    if ((allocated_[index / 8U] & mask) == 0U) {
        return false;
    }
    allocated_[index / 8U] &= static_cast<uint8_t>(~mask);
    std::memcpy(blob, &free_head_, sizeof(free_head_));
    free_head_ = blob;
    return true;
}

} // namespace tools
