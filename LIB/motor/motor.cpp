#include "motor.hpp"
#include "hardware.hpp"

#include <cstring>
#include <utility>

#include "pack.h"
#include "stm32g4xx_hal.h"
#include "uartcomm.hpp"

Motor::Motor(Flash& flash)
    : flash_(flash) {
}

bool Motor::enable(bool enable) {
    enable_motor_ = enable;
    return enable_motor_;
}

bool Motor::init() {
    if (motor_is_ready) {
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
    motor_is_ready = driver_->is_ready();
    return motor_is_ready;
}

bool Motor::is_ready() const {
    return motor_is_ready;
}

bool Motor::is_sensorless_mode() const {
    return sensorless_mode_;
}

bool Motor::fast_loop() {
    if (!enable_motor_)         // 未使能电机，正常退出
        return true;

    if (!motor_is_ready) {
        return false;
    }

    if (!driver_->is_ready()) { // 驱动器不可用，退出
        motor_is_ready = false;
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
    if (sensorless_mode_) {       // 无感模式还要输入电压和电流
        estimater_->set_raw_vol(dcbus_vol, voltage[0], voltage[1], voltage[2]);
        estimater_->set_raw_cur(current[0], current[1], current[2]);
    }

    float estimatered_rad = 0.0f;
    bool success          = estimater_->update(estimatered_rad);
    if (!sensorless_mode_) {      // 有感模式下估计器错误，直接崩溃。否则不用管，开环控制器自动切换
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

    if (fast_look_finished_func_) // 用于实现控制器、编码器热更新
    {
        fast_look_finished_func_();
        fast_look_finished_func_ = nullptr;
    }

    return true;
}

bool Motor::state_update() {
    if (state == State::SETUP)    // 此时处于启动状态，获取当前应该创建的编码器和驱动器型号，以及控制器模式
    {
        flash_.init();              //读入配置信息
        comm_=new UartComm(BSP::uart_comm_port());  //使用串口进行通信
        driver_ = new DRV8301(
            BSP::ic_spi_bus(), BSP::tim1_svpwm(), BSP::adc_measure(), BSP::drv8301_enable_gate(), BSP::drv8301_fault(),
            BSP::drv8301_dc_cal());
        if(driver_==nullptr)    //Driver类构造失败，进入ERROR状态
        {
            core_err=1;
            state=State::ERROR;
            return false;
        }
        
    }
    else if(state==State::ERROR)
    {
        uint8_t buffer[sizeof(Command)+sizeof(uint8_t)];
        Command*pack=(Command*)buffer;
        pack->cmd=MOTOR_ERR;
        pack->size=sizeof(uint8_t);
        pack->data[0]=core_err;
        comm_->send_pack(buffer,sizeof(buffer));
    }
    return true;
}

bool Motor::fast_loop_finished_cb(std::function<void(void)> func) {
    fast_look_finished_func_ = std::move(func);
    return true;
}

void Motor::stop_output() {
    driver_->set_phrase_voltage(0.0f, 0.0f, 0.0f);
    last_voltage_[0] = 0.0f;
    last_voltage_[1] = 0.0f;
    last_voltage_[2] = 0.0f;
}
