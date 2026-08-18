#pragma once

#include <tuple>
#include "FreeRTOS.h"
#include "semphr.h"

class Encoder {
public:
    Encoder();
    float get_value();                     //获取存储的数值
    bool value_is_available(uint32_t us);  //检查数值是否实时可用
    virtual void sensor_update()=0;
    virtual ~Encoder() = default;
protected:
    void record_value(float rad);       //线程安全存储数值和时间戳
private:
    uint32_t encoer_update_ms_;     //上一次编码器更新时间
};
