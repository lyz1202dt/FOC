#pragma once

#include <tuple>
#include "FreeRTOS.h"
#include "semphr.h"

class Encoder {
public:
    Encoder();
    float get_value();              //以线程安全的方式获取数值
    float value_is_available(uint64_t ns);  //检查数值是否实时可用
    virtual void sensor_update()=0;
    virtual ~Encoder() = default;
protected:
    void record_value(float rad);
private:
    uint64_t encoer_update_ms_;     //上一次编码器更新时间
};
