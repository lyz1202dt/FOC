#pragma once

#include <functional>

class Comm {
public:
    Comm();
    virtual void send()=0;
    virtual void register_recv_cb(std::function<bool(void*,int)> cb)=0;
};
