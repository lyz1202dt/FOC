#pragma once

#include <functional>

class Comm {
public:
    Comm();
    virtual ~Comm();
    virtual void register_recv_callback(std::function<void(void* data, int size)> callback);
    virtual bool send_pack(void* data, int size) {
        return false;
    };
};