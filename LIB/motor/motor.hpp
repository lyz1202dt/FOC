#pragma once

#include "encoder.hpp"
#include "controller.hpp"
#include "driver.hpp"
#include "flash.hpp"

class Motor{
public:
    Motor(Flash& flash, Encoder& encoder, MotorDriver& driver, Controller& controller, bool sensorless_mode = false);

    bool init();
    bool is_ready() const;
    bool is_sensorless_mode() const;
    bool fast_loop();  //在PWM更新中断中调用

private:
    void stop_output();

    Flash& flash_;             //闪存控制器，读写配置文件
    Encoder& encoder_;         //编码器控制器，获取编码器数据
    MotorDriver& driver_;      //电机驱动器，获取电流，输出电压
    Controller& controller_;   //FOC控制器，输入电流和编码器状态，输出电压
    bool motor_is_ready{false};
    bool sensorless_mode_{false};
};
