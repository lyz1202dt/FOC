#include "motor.hpp"
#include "hardware.hpp"

Motor::Motor(Flash* flash, Encoder* encoder, MotorDriver* driver, Controller* controller, Estimater* estimater, bool sensorless_mode)
    : flash_(flash)
    , encoder_(encoder)
    , driver_(driver)
    , controller_(controller)
    , estimater_(estimater)
    , sensorless_mode_(sensorless_mode) {
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

    if(fast_look_finished_func_)            //用于实现控制器、编码器热更新
    {
        fast_look_finished_func_();
        fast_look_finished_func_=nullptr;
    }
        
    return true;
}

bool Motor::state_update()
{

    return true;
}

bool Motor::fast_loop_finished_cb(std::function<void(void)> func)
{
    fast_look_finished_func_=std::move(func);
    return true;
}

void Motor::stop_output() {
    driver_->set_phrase_voltage(0.0f, 0.0f, 0.0f);
    last_voltage_[0] = 0.0f;
    last_voltage_[1] = 0.0f;
    last_voltage_[2] = 0.0f;
}
