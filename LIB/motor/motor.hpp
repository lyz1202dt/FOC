#pragma once

#include "encoder.hpp"
#include "controller.hpp"
#include "driver.hpp"
#include "estimater.hpp"
#include "flash.hpp"
#include <functional>

class Motor{
public:
    Motor(Flash* flash,
          Encoder* encoder,
          MotorDriver* driver,
          Controller* controller,
          Estimater* estimater,
          bool sensorless_mode = false);

    bool init();
    bool enable(bool enable);
    bool is_ready() const;
    bool is_sensorless_mode() const;
    bool fast_loop();  //在PWM更新中断中调用
    bool state_update();    //状态更新循环，在线程中调用
    bool fast_loop_finished_cb(std::function<void(void)> func);


    Flash* flash_;             //闪存控制器，读写配置文件
    Encoder* encoder_;         //编码器控制器，获取编码器数据
    MotorDriver* driver_;      //电机驱动器，获取电流，输出电压
    Controller* controller_;   //FOC控制器，输入电流和编码器状态，输出电压
    Estimater* estimater_;     //状态估计器，融合传感器与电压/电流供控制使用

private:
    void stop_output();
    
    float last_voltage_[3]{0.0f, 0.0f, 0.0f};
    bool enable_motor_;
    bool motor_is_ready{false};
    bool sensorless_mode_{false};
    std::function<void(void)> fast_look_finished_func_;
};
