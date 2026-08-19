#pragma once

#include "../encoder/encoder.hpp"
#include "../controller/controller.hpp"
#include "driver.hpp"
#include "../tools/flash.hpp"
#include "comm.hpp"

class Motor{
public:
    Motor();
    bool init();
    bool is_ready() const;
    bool is_sensorless_mode() const;
    bool update();  //在中断中调用
    
    Flash flash;            //闪存控制器，读写配置文件
    Encoder* encoder;       //编码器控制器，获取编码器数据
    MotorDriver* driver;    //电机驱动器，获取电流，输出电压
    Controller* controller; //FOC控制器，输入电流和编码器状态，输出电压
private:
    bool motor_is_ready{false};
    bool sensor_less_mode{false};
};
