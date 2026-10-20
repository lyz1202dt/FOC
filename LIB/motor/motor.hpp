#pragma once

#include "FreeRTOS.h"
#include "encoder.hpp"
#include "controller.hpp"
#include "driver.hpp"
#include "estimater.hpp"
#include "flash.hpp"
#include "queue.h"
#include <cstddef>
#include <cstdint>
#include <functional>
#include "comm.hpp"
#include "memorypoll.hpp"
#include "svpwm.hpp"


class Motor{
public:
    // 注入组件由调用方持有，生命周期须覆盖 Motor 及其定时器回调。
    Motor(Flash& flash, CommManager& comm_manager, BSP::SVPWM& svpwm);

    // 仅在首次调用 init() 前装配；组件由调用方持有，生命周期须覆盖 Motor。
    bool install_driver(Encoder* encoder, MotorDriver* driver);
    bool init();  // 在线程上下文调用；启动 PWM 时基、校准驱动器并注册快环。
    bool enable(bool enable);
    bool fast_loop();  //在PWM更新中断中调用
    bool slow_loop();   //慢循环，维护状态更新，心跳保活包等
    bool handle_event();    //处理事件（在线程上下文执行）

    Flash& flash_;             //闪存控制器，读写配置文件
    CommManager& comm_manager_; //命令输出和状态反馈
    Encoder* encoder_{nullptr};         //编码器控制器，获取编码器数据
    MotorDriver* driver_{nullptr};      //电机驱动器，获取电流，输出电压
    Controller* controller_{nullptr};   //FOC控制器，输入电流和编码器状态，输出电压
    Estimater* estimater_{nullptr};     //状态估计器，融合传感器与电压/电流供控制使用

    Controller* next_controller_{nullptr};   //FOC控制器，输入电流和编码器状态，输出电压
    Estimater* next_estimater_{nullptr};     //状态估计器，融合传感器与电压/电流供控制使用
    bool sensorless_mode_{false};   //控制器运行模式

    enum ControllerType{
        CONTROLLER_IDEL,                  //测试模式
        CONTROLLER_FOC,                 //FOC运行模式
        CONTROLLER_MOTOR_CALIB,         //电机校准模式
        CONTROLLER_ENCODER_CALIB,       //编码器校准模式
        CONTROLLER_TEST,                //测试控制器
        CONTROLLER_NUM
    };

    enum State{
        IDEL,
        TEST,
        FOC,
        ERROR
    };

private:
    bool on_command(Command* data);
    bool notify_error(uint8_t error);
    void stop_output();

    BSP::SVPWM& svpwm_;
    tools::MemoryPoll memory_poll_;
    bool init_started_{false};
    QueueHandle_t event_queue_{nullptr};

    float last_voltage_[3]{0.0f, 0.0f, 0.0f};
    bool enable_motor_{false};
    bool motor_is_ready_{false};
    //Controllers
    Controller* controllers_[ControllerType::CONTROLLER_NUM]{};
    //当前状态
    State state;
    //code
    uint8_t core_error_{0};
};
