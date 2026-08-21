#pragma once

#include "FreeRTOS.h"
#include "flash.hpp"
#include "motor.hpp"
#include <cstring>
#include <functional>
#include <unordered_map>
#include "pack.h"
#include "comm.hpp"

namespace {
    constexpr int kSendbufferSize=128;
}

class PackParser {
    public:
    explicit PackParser(Comm *comm);
    PackParser(PackParser&)=delete;

    template<typename T>
    bool send_command(uint32_t cmd,const T& data){
        constexpr uint32_t data_size=sizeof(T);
        static_assert(data_size+sizeof(Command)<=kSendbufferSize,"PackParser 单次发送的数据包过长");
        Command* pack=(Command*)send_buffer_;
        pack->cmd=cmd;
        pack->size=data_size;
        memcpy(pack->data,&data,data_size);
        return comm_->send_pack(pack, sizeof(Command)+data_size);
    }
    void on_received_pack(void* data,int size);
    void register_cmd_callback(uint32_t cmd,std::function<void(const Command* cmd)>func);   //为每一个cmd提供注册接口
    void unregister_callback(uint32_t cmd);
private:
    SemaphoreHandle_t irq_recv_semphr;
    TaskHandle_t recv_deal_task;
    std::unordered_map<uint32_t, std::function<void(const Command* cmd)>> callbacks;
    uint8_t send_buffer_[kSendbufferSize];
    Comm* comm_;
};
