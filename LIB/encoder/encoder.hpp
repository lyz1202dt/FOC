#pragma once

#include <cstdint>
#include "flash.hpp"

class Encoder {
public:
    explicit Encoder(Flash &flash);
    virtual ~Encoder();
    virtual bool init();                         //启动编码器采样
    virtual bool get_value(float& value, uint32_t max_age_us);  //异步获取转子弧度，并检查这个数据是否足够新鲜
    uint32_t error_;
protected:
    void record_value(float rad);       //线程安全存储数值和时间戳
    Flash &flash;
private:
    uint64_t encoder_update_us_{0};     //上一次编码器更新时间
    float rad_{0.0f};                   //当前转子位置
};
