#pragma once

#include <cstdlib>
#include <functional>

#include "pack.h"

class Comm {
public:
    Comm()          = default;
    virtual ~Comm() = default;
    virtual bool init() {
        return false;
    }
    virtual void register_recv_callback(std::function<void(Command* data)> callback) {
        (void)callback;
    }
    virtual bool send_pack(Command* data) {
        (void)data;
        return false;
    };
    virtual int allow_send() {
        return 0;
    };
};

class CommManager {
public:
    explicit CommManager(int max_channels)
        : max_channels_(max_channels > 0 ? max_channels : 0),
          comm_num_(0),
          p_comm_(max_channels_ > 0 ? new Comm*[max_channels_]{} : nullptr) {}

    ~CommManager() {
        delete[] p_comm_;
    }

    // 禁止拷贝
    CommManager(const CommManager&) = delete;
    CommManager& operator=(const CommManager&) = delete;

    int register_comm_channel(Comm* comm) {
        if (comm == nullptr || comm_num_ >= max_channels_) {
            return -1;
        }

        p_comm_[comm_num_] = comm;
        return comm_num_++;
    }

    bool register_recv_callback(
        std::function<void(Command* data)> callback,
        int channel) {
        if (channel < 0 || channel >= comm_num_ ||
            p_comm_[channel] == nullptr) {
            return false;
        }

        p_comm_[channel]->register_recv_callback(callback);
        return true;
    }

    bool send_pack(Command* data, int channel) {
        if (channel < 0 || channel >= comm_num_ ||
            p_comm_[channel] == nullptr) {
            return false;
        }

        return p_comm_[channel]->send_pack(data);
    }

    int comm_num()const {
        return comm_num_;
    }

    Comm* operator[](int index){
        if(index>=comm_num_)
            return nullptr;
        return p_comm_[index];
    }

private:
    int max_channels_;
    int comm_num_;
    Comm** p_comm_;
};
