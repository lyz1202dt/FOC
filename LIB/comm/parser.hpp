#pragma once

#include "motor.hpp"
#include <functional>
#include <unordered_map>
#include "pack.h"
#include "comm.hpp"

class PackParser {
    public:
    PackParser(Flash *flash,Motor* motor,Comm *comm);
    PackParser(PackParser&)=delete;

    template<typename T>
    bool send_command(uint32_t cmd,const T& data){
        constexpr uint32_t data_size=sizeof(T);
        static_assert(data_size+sizeof(Command)<=64,"PackParser 单次发送的数据包过长");
        Command* pack=(Command*)buffer_;
        pack->cmd=cmd;
        pack->size=data_size;
        memcpy(&data,pack->data,data_size);
        return comm_->send_pack(&cmd, sizeof(T));
    }
    void on_received_pack(void* data,int size);
    void register_callback(uint32_t cmd,std::function<void(const Command& cmd)>func);   //为每一个cmd提供注册接口
    void unregister_callback(uint32_t cmd);
private:
    std::unordered_map<uint32_t, std::function<void(const Command& cmd)>> callbacks;
    uint8_t buffer_[64];
    Comm* comm_;
    Flash* flash_;
    Motor* motor_;
};
