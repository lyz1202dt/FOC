#pragma once

#include "hardware.hpp"
#include <string>
#include <tuple>
#include <utility>

class MotorDriver {
public:

    enum ADCMotor{
        PhraseA,
        PhraseB,
        PhraseC,
        DCBusVol,
    };
    struct{
        uint64_t time;
        uint16_t value;
    }ADCRawValue[4];

    MotorDriver();
    virtual bool init() {                                              // 驱动器初始化
    }
    virtual bool is_ready() {                                          // 驱动器进入正常工作状态
    }
    virtual bool set_enable(bool enable = true) {                      // 开关驱动器功能
        return true;
    }
    
    void set_raw_adc_value(ADCMotor channel,uint16_t value)
    {
        ADCRawValue[channel].value=value;
        ADCRawValue[channel].time=BSP::get_time_us();
    }

    virtual int lookup_reason(std::string& reason) {                   // 查询错误原因并上报
        reason = "Not Set";
        return 0;
    }
    virtual bool get_vbus_voltage(float& vol) {                        // 获取总线电压
        vol = 24.0f;
        return true;
    }
    virtual bool get_phrase_current(float& c1, float& c2, float& c3) { // 计算得到相电流
        return false;
    }
    virtual bool set_phrase_voltage(float v1, float v2, float v3) {    // 输出生成相电压
        return false;
    }
};
