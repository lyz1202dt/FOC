#pragma once

#include <string>
#include "flash.hpp"

class MotorDriver {
public:
    explicit MotorDriver(Flash &flash) : flash(flash) {
    }
    virtual ~MotorDriver() = default;
    virtual bool init() {                                              // 驱动器初始化
        return true;
    }
    virtual bool self_check() {                                              // 驱动器初始化
        return true;
    }
    virtual bool is_ready() {                                          // 驱动器进入正常工作状态
        return false;
    }
    virtual bool set_enable(bool enable = true) {                      // 开关驱动器功能
        return true;
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
    uint32_t error_;
protected:
    Flash &flash;
};
