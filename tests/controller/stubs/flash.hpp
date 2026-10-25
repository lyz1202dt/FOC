#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

class Flash {
public:
    template <typename T>
    bool read_value(const char* key, T& value) {
        ++reads;
        if (!read_ok) {
            return false;
        }
        if (std::strcmp(key, "foc/n") == 0) {
            std::memcpy(&value, &n, sizeof(n));
            return true;
        }
        if (std::strcmp(key, "foc/offset") == 0) {
            std::memcpy(&value, &offset, sizeof(offset));
            return true;
        }
        return false;
    }
    int32_t n{7};
    float offset{0.5f};
    unsigned reads{0};
    bool read_ok{true};
};
