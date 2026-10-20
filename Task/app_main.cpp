#include "app_main.hpp"
#include "as5047p.hpp"
#include "as5600.hpp"
#include "comm.hpp"
#include "driver.hpp"
#include "drv8301.hpp"
#include "encoder.hpp"
#include "estimater.hpp"
#include "hardware.hpp"
#include "i2c.hpp"
#include "pack.h"
#include "projdefs.h"
#include "spi.hpp"
#include "svpwm.hpp"


#include "comm.hpp"
#include "flash.hpp"
#include "motor.hpp"
#include "uartcomm.hpp"

#include <atomic>
#include <string>

TaskHandle_t motor_slow_loop_task_handle;
void motor_slow_loop(void* param);

// 存储配置数据
static Flash flash;
void load_components_from_config(Encoder*& encoder, MotorDriver*& driver);

// 组装电机控制器
CommManager comm_manager(2);
Motor motor(flash, comm_manager);

UartComm uart_comm(BSP::uart_comm_port());

// float debug_rad;
extern "C" void app_main(void* param) {
    BSP::time_init();
    BSP::adc_measure_start();
    BSP::ic_spi_bus().init();
    BSP::sensor_spi_bus().init();
    BSP::sensor_i2c_bus().init();

    flash.init();

    //根据Flash配置，构造当前要使用的组件
    Encoder* encoder;
    MotorDriver* driver;
    load_components_from_config(encoder,driver);
    motor.install_driver(encoder, driver);

    //组织通信接口
    uart_comm.init();
    comm_manager.register_comm_channel(&uart_comm);     //注册唯一的一个串口交互设备

    //准备与DRV8301的通信
    BSP::drv8301_enable_gate().write(false);
    BSP::drv8301_dc_cal().write(false);
    BSP::ic_spi_cs().write(true);

    //电机初始化
    motor.init();

    //DRV8301启动，电机电流检测校准
    BSP::tim1_svpwm().init();
    BSP::tim1_svpwm().enable_output(false);
    driver->self_check();   //完成自检，电流零偏校准(ADC由定时器触发，所以必须在TIM1开启后启动自检)
    BSP::tim1_svpwm().enable_output(true);
    
    //创建电流块环任务
    BSP::tim1_svpwm().register_timer_update_cb([]() {
        motor.fast_loop();
    });

    motor.enable(true);
    
    //创建状态更新环路
    xTaskCreate(motor_slow_loop, "motor_slow_loop", 256, NULL, 1, &motor_slow_loop_task_handle);
    //创建事件处理环路
    while (1) {
        motor.handle_event(); // 阻塞等待并处理电机事件
    }
}

void motor_slow_loop(void* param) {
    while (1) {
        motor.slow_loop();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// 启动阶段调用一次，构建的组件在整个应用生命周期内供 motor 使用。
void load_components_from_config(Encoder*& encoder, MotorDriver*& driver) {
    int encoder_type = 0;
    flash.read_value("encoder", encoder_type);
    switch (encoder_type) {
    case 0: encoder = new AS5600(flash, BSP::sensor_i2c_bus()); break;
    case 1: encoder = new AS5047P(flash); break;
    default: encoder = nullptr; break;
    }

    int driver_type = 0; // 未配置时沿用 DRV8301。
    flash.read_value("driver", driver_type);
    switch (driver_type) {
    case 0:
        driver = new DRV8301(
            flash, BSP::ic_spi_bus(), BSP::tim1_svpwm(), BSP::adc_measure(), BSP::drv8301_enable_gate(), BSP::drv8301_fault(),
            BSP::drv8301_dc_cal());
        break;
    default: driver = nullptr; break;
    }
}
