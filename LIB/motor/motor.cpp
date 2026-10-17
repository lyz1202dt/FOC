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

Motor::Motor(Flash& flash, CommManager& comm_manager)
    : flash_(flash)
    , comm_manager_(comm_manager)
    , memory_poll_(32,6) {
}

bool Motor::enable(bool enable) {
    enable_motor_ = enable;
    return enable_motor_;
}

bool Motor::install_driver(Encoder* encoder, MotorDriver* driver) {
    if (init_started_ || encoder == nullptr || driver == nullptr) {
        return false;
    }
    encoder_ = encoder;
    driver_ = driver;
    return true;
}

bool Motor::init() {
    init_started_ = true;
    event_queue_=xQueueCreate(8, sizeof(void*));

    for (int i = 0; i < comm_manager_.comm_num(); i++) { // 为所有信道来源注册命令接收回调
        if (!comm_manager_.register_recv_callback(
                [this](Command* data) {
                    on_command(data);
                },
                i)) {
            core_error_=0x21;
            return false;
        }
    }

    if (driver_ == nullptr) {                           // 应用未装配有效的驱动器
        core_error_=0x01;
        return false;
    }

    if (encoder_ == nullptr) {                          // 应用未装配有效的编码器
        core_error_=0x02;
        return false;
    }

    int estimater_type = 0;
    flash_.read_value("estimater", estimater_type);     // 加载配置指定的Estimater
    switch (estimater_type) {                           // 根据配置的编码器读取数据
    case 0:                                             // 默认估计器
        estimater_ = new Estimater(flash_);
        break;
    default: estimater_ = nullptr;
    }
    if (estimater_ == nullptr) {                        // Estimater类构造失败，进入ERROR状态
        core_error_=0x03;
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
    next_estimater_ = estimater_;

    if (controller_ == nullptr) {
        core_error_=0x04;
        return false;
    }

    if (!sensorless_mode_ && !encoder_->init()) {
        core_error_=0x05;
        return false;
    }

    if (!driver_->init()) {
        core_error_=0x06;
        return false;
    }
    if (!driver_->set_enable(true)) {
        core_error_=0x07;
        return false;
    }
    if (!driver_->is_ready()) {
        core_error_=0x08;
        return false;
    }

    return true;
}

bool Motor::fast_loop() {
    if (!enable_motor_)         // 未使能电机，正常退出
        return true;

    if(core_error_) //错误码没有清除，不执行控制
        return false;

    if (!motor_is_ready_) { 
        core_error_=0x10;
        notify_error(core_error_);
        return false;
    }
    if (!driver_->is_ready()) { // 驱动器不可用，退出
        core_error_=0x20;
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

    // 检查控制器和估计器热切换
    if (next_estimater_ && (next_estimater_ != estimater_))
        estimater_ = next_estimater_;
    if (next_controller_ && (next_controller_ != controller_))
        controller_ = next_controller_;

    return true;
}

void Motor::stop_output() {
    driver_->set_phrase_voltage(0.0f, 0.0f, 0.0f);
    last_voltage_[0] = 0.0f;
    last_voltage_[1] = 0.0f;
    last_voltage_[2] = 0.0f;
}

bool Motor::handle_event() {
    Command* command=nullptr;
    xQueueReceive(event_queue_, &command, portMAX_DELAY);
    uint32_t cmd=command->cmd;
    if(cmd==0x01)   //启动/停止控制输出fast_loop
    {
        enable(command->data[0]?true:false);
    }
    else if(cmd==0x02)  //请求切入IDEL模式
    {
        state=State::IDEL;
        next_controller_ = controllers_[ControllerType::CONTROLLER_IDEL];
    }
    else if(cmd==0x03)  //请求切入开环测试
    {
        state=State::TEST;
        next_controller_ = controllers_[ControllerType::CONTROLLER_TEST];
    }
    else if(cmd==0x04)  //请求切入FOC闭环
    {
        state=State::FOC;
        next_controller_ = controllers_[ControllerType::CONTROLLER_FOC];
    }
    else if(cmd==0x05)  //设置FOC参考力矩电流
    {
        controllers_[ControllerType::CONTROLLER_FOC]->controller_reference(command->data, sizeof(float));
    }
    else if(cmd==0x06)  //请求获取电机状态
    {
        //TODO:触发复位重启（还需要增加一些关断驱动器的代码）
        //NVIC_SystemReset();
    }
    else if(cmd==0x07)  //设置参数值(存储的格式为:size(uint8_t)|c_str|data)
    {
        constexpr size_t kMaxPayloadSize = 32U - sizeof(Command);
        if (command->size < 4U || command->size > kMaxPayloadSize) {
            memory_poll_.release_blob(command);
            return false;
        }
        const size_t value_length = command->data[0];
        const char* key = reinterpret_cast<const char*>(command->data + 1U);
        const char* key_end = static_cast<const char*>(
            std::memchr(key, '\0', command->size - 1U));
        if (key_end == nullptr || key_end == key || value_length == 0U) {
            memory_poll_.release_blob(command);
            return false;
        }
        const size_t value_offset = 1U + static_cast<size_t>(key_end - key) + 1U;
        if (value_length != command->size - value_offset ||
            !flash_.write_raw(key, command->data + value_offset, value_length)) {
            memory_poll_.release_blob(command);
            return false;
        }
    }
    else if(cmd==0x08)  //请求参数值,向所有通道发送一次参数的值
    {
        constexpr size_t kMaxPayloadSize = 32U - sizeof(Command);
        if (command->size < 3U || command->size > kMaxPayloadSize) {
            memory_poll_.release_blob(command);
            return false;
        }
        const size_t value_length = command->data[0];
        const char* key = reinterpret_cast<const char*>(command->data + 1U);
        const char* key_end = static_cast<const char*>(
            std::memchr(key, '\0', command->size - 1U));
        if (key_end == nullptr || key_end == key || value_length == 0U) {
            memory_poll_.release_blob(command);
            return false;
        }
        const size_t value_offset = 1U + static_cast<size_t>(key_end - key) + 1U;
        if (value_offset != command->size ||
            value_length > kMaxPayloadSize - value_offset ||
            !flash_.read_raw(key, command->data + value_offset, value_length)) {
            memory_poll_.release_blob(command);
            return false;
        }
        command->size = static_cast<uint32_t>(value_offset + value_length);
        bool sent = true;
        for (int channel = 0; channel < comm_manager_.comm_num(); ++channel) {
            if (!comm_manager_.send_pack(
                    command, channel)) {
                sent = false;
            }
        }
        memory_poll_.release_blob(command);
        return sent;
    }
    else if(cmd==0x09)  //保存参数值到flash中
    {
        flash_.fresh();
    }
    else if(cmd==0x0A)  //触发复位
    {
        //TODO:触发复位重启，还需要增加一些安全处理的代码
    }
    else if(cmd==0x0A)  //清除错误信息
    {
        core_error_=0;
    }
    else if(cmd==0xFF)  //驱动器错误
    {
        next_controller_=controller_=controllers_[ControllerType::CONTROLLER_IDEL];
        state=State::ERROR;
    }
    memory_poll_.release_blob(command);
    return true;
}

bool Motor::slow_loop()
{
    if(state==State::ERROR)
    {
        //TODO:提示错误信息
    }
    else if(state==FOC)
    {
        //TODO:指示FOC正常运转
    }
    return true;
}

//必须在IRQ上下文调用
bool Motor::notify_error(uint8_t error){
    Command command{.cmd=0xFF,.size=0};
    return on_command(&command);
}

bool Motor::on_command(Command* data) //注意：位于ISR上下文
{
    if (data == nullptr || data->size > 32U - sizeof(Command)) {
        return false;
    }
    const size_t size = sizeof(Command) + data->size;
    void* cmd_block=memory_poll_.require_blob();
    if(cmd_block==nullptr)  //内存池空间不足，直接退出
        return false;
    memcpy(cmd_block,data,size);
    BaseType_t pxHigherPriorityTaskWoken=pdFALSE;
    xQueueSendFromISR(event_queue_, cmd_block, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    return true;
}
