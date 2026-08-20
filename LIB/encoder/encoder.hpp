#pragma once

#include <cstdint>

class Encoder {
public:
    Encoder();
    virtual ~Encoder();
    virtual bool init();                         //启动编码器采样
    float get_value() const;                     //获取转子弧度
    bool get_value(float& value, uint32_t max_age_us) const;
    bool value_is_available(uint32_t max_age_us) const;  //检查数值是否实时可用
protected:
    void record_value(float rad);       //线程安全存储数值和时间戳
private:
    uint64_t encoder_update_us_{0};     //上一次编码器更新时间
    float rad_{0.0f};                   //当前转子位置
};
