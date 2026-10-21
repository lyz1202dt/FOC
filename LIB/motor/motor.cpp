#include "motor.hpp"
#include "comm.hpp"
#include "estimater.hpp"
#include "flash.hpp"
#include "foc.hpp"
#include "hardware.hpp"
#include "idel.hpp"
#include "pack.h"
#include "projdefs.h"
#include "voc.hpp"
#include <cstring>

Motor::Motor(Flash& flash, CommManager& comm_manager, BSP::SVPWM& svpwm)
    : flash_(flash)
    , comm_manager_(comm_manager)
    , svpwm_(svpwm)
    , memory_poll_(32, 6) {
}

bool Motor::enable(bool enable) {
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    enable_motor_ = false;
    bool success  = false;
    if (driver_ != nullptr) {
        if (enable && motor_is_ready_ && core_error_ == 0U) {
            last_voltage_[0] = last_voltage_[1] = last_voltage_[2] = 0.0f;
            enable_motor_                                          = driver_->enable_pwm(true);
            success                                                = enable_motor_;
        } else {
            success = driver_->enable_pwm(false) && !enable;
        }
    }
    __set_PRIMASK(interrupt_mask);
    return success;
}

bool Motor::install_driver(Encoder* encoder, MotorDriver* driver) {
    if (init_started_ || encoder == nullptr || driver == nullptr) {
        return false;
    }
    encoder_ = encoder;
    driver_  = driver;
    return true;
}

bool Motor::init() {
    if (init_started_) {
        return motor_is_ready_ && core_error_ == 0U;
    }
    init_started_ = true;
    event_queue_  = xQueueCreate(8, sizeof(void*));
    if (event_queue_ == nullptr) {
        core_error_ = 0x09;
        return false;
    }

    for (int i = 0; i < comm_manager_.comm_num(); i++) { // 为所有信道来源注册命令接收回调
        if (!comm_manager_.register_recv_callback(
                [this](Command* data) {
                    on_command(data);
                },
                i)) {
            core_error_ = 0x21;
            return false;
        }
    }

    if (driver_ == nullptr) {                            // 未装配有效的驱动器
        core_error_ = 0x01;
        return false;
    }

    if (encoder_ == nullptr) {                           // 未装配有效的编码器
        core_error_ = 0x02;
        return false;
    }

    int estimater_type = 0;
    flash_.read_value("components/estimater", estimater_type);      // 加载配置指定的Estimater
    switch (estimater_type) {                            // 根据配置的编码器读取数据
    case 0:                                              // 默认估计器
        estimater_ = new Estimater(flash_);
        break;
    default: estimater_ = nullptr;
    }
    if (estimater_ == nullptr) {                         // Estimater类构造失败，进入ERROR状态
        core_error_ = 0x03;
        return false;
    }

    // 创建全部控制器
    controllers_[ControllerType::CONTROLLER_IDEL]          = new class IDEL(flash_);
    controllers_[ControllerType::CONTROLLER_FOC]           = new class FOC(flash_);
    controllers_[ControllerType::CONTROLLER_TEST]          = new class VOC(flash_);
    controllers_[ControllerType::CONTROLLER_MOTOR_CALIB]   = nullptr;
    controllers_[ControllerType::CONTROLLER_ENCODER_CALIB] = nullptr;
    // 设置初始控制器为IDEL
    controller_ = next_controller_ = controllers_[ControllerType::CONTROLLER_IDEL];
    next_estimater_                = estimater_;

    if (controller_ == nullptr) {
        core_error_ = 0x04;
        return false;
    }

    if (!sensorless_mode_ && !encoder_->init()) {
        core_error_ = 0x05;
        return false;
    }

    if (!driver_->init()) {
        core_error_ = 0x06;
        return false;
    }
    if (!driver_->is_ready()) {
        core_error_ = 0x08;
        return false;
    }

    svpwm_.register_timer_update_cb([this]() {
        fast_loop();
    });
    if (!driver_->enable_measure(true)) { // 开始ADC测量
        core_error_ = 0x07;
        return false;
    }
    svpwm_.enable_timer(true);            // 启动定时器，触发ADC采样
    if (!driver_->self_check()) {         // 自检，校准电流采样
        core_error_ = 0x0B;
        return false;
    }
    motor_is_ready_ = true;
    return true;
}

bool Motor::fast_loop() {
    if (core_error_)                      // 错误码没有清除，不执行控制
        return false;

    if (!motor_is_ready_) {
        // core_error_ = 0x10;
        // notify_error(core_error_);
        return true;
    }
    
    if (next_estimater_ && next_estimater_ != estimater_) {
        estimater_ = next_estimater_;
    }
    if (next_controller_ && next_controller_ != controller_) {
        controller_ = next_controller_;
        controller_->reset();
    }
    state = next_state;

    if (!driver_->is_ready()) {           // 驱动器不可用，退出
        core_error_ = 0x20;
        notify_error(core_error_);
        return false;
    }

    float dcbus_vol;
    float current[3];
    float voltage[3]{last_voltage_[0], last_voltage_[1], last_voltage_[2]};
    if (!driver_->get_vbus_voltage(dcbus_vol) || !driver_->get_phrase_current(current[0], current[1], current[2])) {
        stop_output();
        return false;
    }

    float encoder_rad = 0.0f;
    const bool encoder_available = encoder_->get_value(encoder_rad, 2000U);
    estimater_->set_raw_rad(encoder_rad, encoder_available);
    // 估计器输入
    if (sensorless_mode_) {  // 无感模式还要输入电压和电流
        estimater_->set_raw_vol(dcbus_vol, voltage[0], voltage[1], voltage[2]);
        estimater_->set_raw_cur(current[0], current[1], current[2]);
    }

    float estimatered_rad = 0.0f;
    float estimatered_omega = 0.0f;
    bool success = estimater_->update();
    success = estimater_->get_value(estimatered_rad, estimatered_omega) && success;
    if (!sensorless_mode_) { // 有感模式下估计器错误，直接崩溃。否则不用管，开环控制器自动切换
        if (!success) {
            stop_output();
            return false;
        }
    }

    if (!enable_motor_) // 未使能电机，正常退出，不执行控制
    {
        driver_->set_phrase_voltage(0.0f, 0.0f, 0.0f);
        return true;
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

    return true;
}

void Motor::stop_output() {
    driver_->set_phrase_voltage(0.0f, 0.0f, 0.0f);
    last_voltage_[0] = 0.0f;
    last_voltage_[1] = 0.0f;
    last_voltage_[2] = 0.0f;
}

void Motor::request_controller(ControllerType type, State requested_state) {
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    next_controller_ = controllers_[type];
    next_state = requested_state;
    __set_PRIMASK(interrupt_mask);
}

bool Motor::handle_event() {
    Command* command = nullptr;
    xQueueReceive(event_queue_, &command, portMAX_DELAY);
    uint32_t cmd = command->cmd;
    if (cmd == 0x01)          // 启动/停止控制输出fast_loop
    {
        enable(command->data[0] ? true : false);
    } else if (cmd == 0x02)   // 请求切入IDEL模式
    {
        request_controller(ControllerType::CONTROLLER_IDEL, State::IDEL);
    } else if (cmd == 0x03)   // 请求切入开环测试
    {
        if (core_error_ == 0) // 无错误允许切换
        {
            request_controller(ControllerType::CONTROLLER_TEST, State::TEST);
        }
    } else if (cmd == 0x04)   // 请求切入FOC闭环
    {
        if (core_error_ == 0) // 无错误允许切换
        {
            request_controller(ControllerType::CONTROLLER_FOC, State::FOC);
        }
    } else if (cmd == 0x05)   // 设置FOC参考力矩电流
    {
        controllers_[ControllerType::CONTROLLER_FOC]->controller_reference(command->data, sizeof(float));
    } else if (cmd == 0x06)   // 请求获取电机状态
    {
        uint8_t send_buffer[sizeof(Command)+sizeof(MotorStatus)];
        Command* pack=(Command*)send_buffer;
        pack->cmd=0x16;
        pack->size=sizeof(MotorStatus);
        MotorStatus* payload=(MotorStatus*)pack->data;
        controllers_[ControllerType::CONTROLLER_FOC]->controller_status(&(payload->current), sizeof(float));
        estimater_->get_value(payload->position, payload->velocity);
        payload->status=0;  //TODO:填写状态/错误码
        comm_manager_.send_pack(pack, 0);   //默认反馈到0通道
    } else if (cmd == 0x07) // 设置参数值(存储的格式为:size(uint8_t)|c_str|data)
    {
        constexpr size_t kMaxPayloadSize = 32U - sizeof(Command);
        if (command->size < 4U || command->size > kMaxPayloadSize) {
            memory_poll_.release_blob(command);
            return false;
        }
        const size_t value_length = command->data[0];
        const char* key           = reinterpret_cast<const char*>(command->data + 1U);
        const char* key_end       = static_cast<const char*>(std::memchr(key, '\0', command->size - 1U));
        if (key_end == nullptr || key_end == key || value_length == 0U) {
            memory_poll_.release_blob(command);
            return false;
        }
        const size_t value_offset = 1U + static_cast<size_t>(key_end - key) + 1U;
        if (value_length != command->size - value_offset || !flash_.write_raw(key, command->data + value_offset, value_length)) {
            memory_poll_.release_blob(command);
            return false;
        }
    } else if (cmd == 0x08) // 请求参数值,向所有通道发送一次参数的值
    {
        constexpr size_t kMaxPayloadSize = 32U - sizeof(Command);
        if (command->size < 3U || command->size > kMaxPayloadSize) {
            memory_poll_.release_blob(command);
            return false;
        }
        const size_t value_length = command->data[0];
        const char* key           = reinterpret_cast<const char*>(command->data + 1U);
        const char* key_end       = static_cast<const char*>(std::memchr(key, '\0', command->size - 1U));
        if (key_end == nullptr || key_end == key || value_length == 0U) {
            memory_poll_.release_blob(command);
            return false;
        }
        const size_t value_offset = 1U + static_cast<size_t>(key_end - key) + 1U;
        if (value_offset != command->size || value_length > kMaxPayloadSize - value_offset
            || !flash_.read_raw(key, command->data + value_offset, value_length)) {
            memory_poll_.release_blob(command);
            return false;
        }
        command->size = static_cast<uint32_t>(value_offset + value_length);
        bool sent     = true;
        for (int channel = 0; channel < comm_manager_.comm_num(); ++channel) {
            if (!comm_manager_.send_pack(command, channel)) {
                sent = false;
            }
        }
        memory_poll_.release_blob(command);
        return sent;
    } else if (cmd == 0x09) // 保存参数值到flash中
    {
        if (!flash_.fresh()) {
            memory_poll_.release_blob(command);
            return false;
        }
    } else if (cmd == 0x0A) // 清除错误信息
    {
        core_error_ = 0;
    } else if (cmd == 0xFF) // 驱动器错误
    {
        const uint32_t interrupt_mask = __get_PRIMASK();
        __disable_irq();
        next_controller_ = controller_ = controllers_[ControllerType::CONTROLLER_IDEL];
        controller_->reset();
        next_state = state = State::ERROR;
        __set_PRIMASK(interrupt_mask);
    }
    memory_poll_.release_blob(command);
    return true;
}

// LED指示，按键扫描
bool Motor::slow_loop() {
    if (state == State::ERROR) {

        // TODO:提示错误信息
    } else if (state == FOC) {
        // TODO:指示FOC正常运转
    } else if (state == State::IDEL) {
        // 发送心跳包指示设备就绪
        
    } else if (state == TEST) {
        // TODO:电压开环测试状态
    }

    //一般的任务
    Command ok_pack = {.cmd = 0x5A, .size = 0};
    for (int i = 0; i < comm_manager_.comm_num(); i++)
    {
        if(comm_manager_[i]->allow_send()==0)  //此时总线允许发送
            comm_manager_.send_pack(&ok_pack, i);
    }
        

    slow_loop_cnt++;
    return true;
}

// 必须在IRQ上下文调用
bool Motor::notify_error(uint8_t error) {
    Command command{.cmd = 0xFF, .size = 0};
    return on_command(&command);
}

bool Motor::on_command(Command* data) // 注意：位于ISR上下文
{
    if (data == nullptr || data->size > 32U - sizeof(Command)) {
        return false;
    }
    const size_t size = sizeof(Command) + data->size;
    void* cmd_block   = memory_poll_.require_blob();
    if (cmd_block == nullptr)         // 内存池空间不足，直接退出
        return false;
    memcpy(cmd_block, data, size);
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(event_queue_, &cmd_block, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    return true;
}
