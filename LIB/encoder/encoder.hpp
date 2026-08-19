#pragma once

#include <tuple>
#include "FreeRTOS.h"
#include "semphr.h"

class Encoder {
public:
    Encoder();
    float get_value();                     //获取转子弧度
    bool value_is_available(const uint32_t &us);  //检查数值是否实时可用
protected:
    void record_value(float rad);       //线程安全存储数值和时间戳
private:

    uint64_t encoer_update_us_;     //上一次编码器更新时间
    float rad;      //当前转子位置
    int r;          //自启动起，转过的圈数
};
