#pragma once

#include <functional>

class Comm {
public:
    Comm()          = default;
    virtual ~Comm() = default;
    virtual void register_recv_callback(std::function<void(void* data, int size)> callback) {
        (void)callback;
    }
    virtual bool send_pack(void* data, int size) {
        (void)data;
        (void)size;
        return false;
    };
};
