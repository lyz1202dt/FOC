#pragma once

#include "FreeRTOS.h"
#include "semphr.h"
#include <cstddef>
#include <string>
#include <type_traits>

class Flash {
public:
    Flash();

    template <typename T>
    bool write_value(std::string key, const T &value) {
        static_assert(std::is_trivially_copyable<T>::value,
                      "Flash only supports trivially copyable value types");
        return write_raw(key, &value, sizeof(T));
    }

    template <typename T>
    bool read_value(std::string key, T &value) {
        static_assert(std::is_trivially_copyable<T>::value,
                      "Flash only supports trivially copyable value types");
        return read_raw(key, &value, sizeof(T));
    }

    static SemaphoreHandle_t write_mutex;
    static SemaphoreHandle_t read_mutex;

private:
    static bool write_raw(const std::string &key, const void *value, size_t length);
    static bool read_raw(const std::string &key, void *value, size_t length);
};
