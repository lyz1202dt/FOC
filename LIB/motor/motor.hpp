#pragma once

#include "encoder.hpp"
#include "controller.hpp"
#include "driver.hpp"
#include "estimater.hpp"
#include "flash.hpp"
#include <cstddef>
#include <functional>
#include "comm.hpp"

#include "as5600.hpp"
#include "drv8301.hpp"
#include "pack.h"

class Motor{
public:
    Motor(Flash& flash,CommManager& comm_manager);

    bool init();
    bool enable(bool enable);
    bool is_sensorless_mode() const;
    bool fast_loop();  //在PWM更新中断中调用
    bool state_update();    //状态更新循环，在线程中调用

    Flash& flash_;             //闪存控制器，读写配置文件
    CommManager& comm_manager_; //命令输出和状态反馈
    Encoder* encoder_;         //编码器控制器，获取编码器数据
    MotorDriver* driver_;      //电机驱动器，获取电流，输出电压
    Controller* controller_;   //FOC控制器，输入电流和编码器状态，输出电压
    Estimater* estimater_;     //状态估计器，融合传感器与电压/电流供控制使用

    Controller* next_controller_;   //FOC控制器，输入电流和编码器状态，输出电压
    Estimater* next_estimater_;     //状态估计器，融合传感器与电压/电流供控制使用
    bool sensorless_mode_{false};   //控制器运行模式

    enum State{
        SETUP,                  //启动模式
        IDEL,                   //IDEL模式
        MOTOR_CALIBRATION,      //电机校准模式
        ENCODER_CALIBRATION,    //编码器校准模式
        RUNNING,                //FOC运行模式
        ERROR,                  //测试模式
        TEST,                   //IDEL模式
        STATE_NUM
    };

    enum ControllerType{
        CONTROLLER_IDEL,                  //测试模式
        CONTROLLER_FOC,                 //FOC运行模式
        CONTROLLER_MOTOR_CALIB,         //电机校准模式
        CONTROLLER_ENCODER_CALIB,       //编码器校准模式
        CONTROLLER_TEST,                //测试控制器
        CONTROLLER_NUM
    };

private:
    bool on_command(void* data,int size);
    void stop_output();
    float last_voltage_[3]{0.0f, 0.0f, 0.0f};
    bool enable_motor_{false};
    bool motor_is_ready_{false};
    State state_{State::SETUP};
    State req_state_{State::SETUP};

    //Controllers
    Controller* controllers_[ControllerType::CONTROLLER_NUM];
    //code
    uint8_t core_err_{0};
};
