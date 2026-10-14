#include "motor.hpp"
#include "as5047p.hpp"
#include "as5600.hpp"
#include "comm.hpp"
#include "estimater.hpp"
#include "foc.hpp"
#include "hardware.hpp"
#include "idel.hpp"
#include "pack.h"
#include "voc.hpp"
#include <cstring>

Motor::Motor(Flash& flash, CommManager& comm_manager)
    : flash_(flash)
    , comm_manager_(comm_manager) {
}

bool Motor::enable(bool enable) {
    enable_motor_ = enable;
    return enable_motor_;
}

bool Motor::init() {
    if (motor_is_ready_) {
        return true;
    }

    if (!sensorless_mode_ && !encoder_->init()) {
        return false;
    }

    if (!driver_->init()) {
        return false;
    }
    if (!driver_->set_enable(true)) {
        return false;
    }
    motor_is_ready_ = driver_->is_ready();
    return motor_is_ready_;
}

bool Motor::is_sensorless_mode() const {
    return sensorless_mode_;
}

bool Motor::fast_loop() {
    if (!enable_motor_)         // 未使能电机，正常退出
        return true;

    if (!motor_is_ready_) {
        state_    = State::ERROR;
        core_err_ = 0x10;
        return false;
    }

    if (!driver_->is_ready()) { // 驱动器不可用，退出
        motor_is_ready_ = false;
        stop_output();
        return false;
    }

    float dcbus_vol;
    float current[3];
    float voltage[3]{last_voltage_[0], last_voltage_[1], last_voltage_[2]};
    if (!driver_->get_vbus_voltage(dcbus_vol) || !driver_->get_phrase_current(current[0], current[1], current[2])) {
        stop_output();
        return false;
    }

    // 估计器输入
    estimater_->set_raw_rad(encoder_->get_value(), encoder_->value_is_available(2000));
    if (sensorless_mode_) {  // 无感模式还要输入电压和电流
        estimater_->set_raw_vol(dcbus_vol, voltage[0], voltage[1], voltage[2]);
        estimater_->set_raw_cur(current[0], current[1], current[2]);
    }

    float estimatered_rad = 0.0f;
    bool success          = estimater_->update(estimatered_rad);
    if (!sensorless_mode_) { // 有感模式下估计器错误，直接崩溃。否则不用管，开环控制器自动切换
        if (!success) {
            stop_output();
            return false;
        }
    }

    controller_->set_measure(current[0], current[1], current[2], dcbus_vol, estimatered_rad, success);
    if (!controller_->get_output(voltage[0], voltage[1], voltage[2])) {
        stop_output();
        return false;
    }
    if (!driver_->set_phrase_voltage(voltage[0], voltage[1], voltage[2])) {
        return false;
    }
    last_voltage_[0] = voltage[0];
    last_voltage_[1] = voltage[1];
    last_voltage_[2] = voltage[2];

    // 检查控制器切换
    if (next_estimater_ && (next_estimater_ != estimater_))
        estimater_ = next_estimater_;
    if (next_controller_ && (next_controller_ != controller_))
        controller_ = next_controller_;

    return true;
}

bool Motor::state_update() {
    if (state_ == State::SETUP)                         // 此时处于启动状态，获取当前应该创建的编码器和驱动器型号，以及控制器模式
    {
        flash_.init();                                  // 构造Flash表
        for (int i = 0; i < comm_manager_.comm_num(); i++)  //为所有信道来源注册命令接收回调
            comm_manager_.register_recv_callback(
                [this](void* data, int size) {
                    on_command(data, size);
                },
                i);
        
        driver_ = new DRV8301(
            BSP::ic_spi_bus(), BSP::tim1_svpwm(), BSP::adc_measure(), BSP::drv8301_enable_gate(), BSP::drv8301_fault(),
            BSP::drv8301_dc_cal());
        if (driver_ == nullptr)                         // Driver类构造失败，进入ERROR状态
        {
            core_err_ = 1;
            state_    = State::ERROR;
            return false;
        }

        int encoder_type = -1;
        flash_.read_value("encoder", encoder_type);     // 加载配置指定的Encoder
        switch (encoder_type) {                         // 根据配置的编码器读取数据
        case 0: encoder_ = new AS5600(BSP::sensor_i2c_bus()); break;
        case 1: encoder_ = new AS5047P(); break;
        default: encoder_ = nullptr;
        }
        if (encoder_ == nullptr)                        // Encoder类构造失败，进入ERROR状态
        {
            core_err_ = 2;
            state_    = State::ERROR;
            return false;
        }


        int estimater_type = 0;
        flash_.read_value("estimater", estimater_type); // 加载配置指定的Estimater
        switch (estimater_type) {                       // 根据配置的编码器读取数据
        case 0:                                         // 默认估计器
            estimater_ = new Estimater(flash_);
            break;
        default: estimater_ = nullptr;
        }
        if (estimater_ == nullptr)                      // Estimater类构造失败，进入ERROR状态
        {
            core_err_ = 3;
            state_    = State::ERROR;
            return false;
        }

        // 创建全部控制器
        controllers_[ControllerType::CONTROLLER_IDEL]          = new class IDEL(flash_);
        controllers_[ControllerType::CONTROLLER_FOC]           = new class FOC();
        controllers_[ControllerType::CONTROLLER_TEST]          = new class VOC(flash_);
        controllers_[ControllerType::CONTROLLER_MOTOR_CALIB]   = nullptr;
        controllers_[ControllerType::CONTROLLER_ENCODER_CALIB] = nullptr;
        // 设置初始控制器为IDEL
        controller_ = next_controller_ = controllers_[ControllerType::CONTROLLER_IDEL];

        motor_is_ready_ = true;                             // 电机ready，切入IDEL模式
        state_          = State::IDEL;
    } else if (state_ == State::IDEL)                       // IDEL状态
    {

    } else if (state_ == State::TEST)                       // VOC 测试状态
    {

    } else if (state_ == State::ERROR)                      // 错误状态，报告错误
    {
        uint8_t buffer[sizeof(Command) + sizeof(uint8_t)];
        Command* pack = (Command*)buffer;
        pack->cmd     = MOTOR_ERR;
        pack->size    = sizeof(uint8_t);
        pack->data[0] = core_err_;
        comm_manager_.send_pack(buffer, sizeof(buffer), 0); // 使用信道0发送错误信息
    }
    return true;
}

void Motor::stop_output() {
    driver_->set_phrase_voltage(0.0f, 0.0f, 0.0f);
    last_voltage_[0] = 0.0f;
    last_voltage_[1] = 0.0f;
    last_voltage_[2] = 0.0f;
}

bool Motor::on_command(void* data,int size)
{
    Command* cmd_pack=static_cast<Command*>(data);
    if(cmd_pack->size+sizeof(Command)!=size)
        return false;
    uint32_t cmd=cmd_pack->cmd;
    if(cmd==0x01)   //电机使能指令
    {
        enable(cmd_pack->data[0]?true:false);
    }
    else if(cmd==0x02)  //请求开环测试
    {
        req_state_=State::IDEL;
    }
    else if(cmd==0x03)  //设置FOC参考力矩电流
    {
        controllers_[ControllerType::CONTROLLER_FOC]->controller_reference((float*)cmd_pack->data, sizeof(float));
    }
}
