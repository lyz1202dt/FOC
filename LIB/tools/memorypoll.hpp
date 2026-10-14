#pragma once

#include <cstddef>
#include <cstdint>

namespace tools {

class MemoryPoll {
public:
    MemoryPoll(std::size_t blob_size, std::size_t blob_count);
    ~MemoryPoll();

    MemoryPoll(const MemoryPoll&) = delete;
    MemoryPoll& operator=(const MemoryPoll&) = delete;

    void* require_blob();
    bool release_blob(void*);

private:
    uint8_t* storage_ = nullptr;
    uint8_t* allocated_ = nullptr;
    void* free_head_ = nullptr;
    std::size_t blob_size_ = 0U;
    std::size_t blob_count_ = 0U;
};

} // namespace tools
