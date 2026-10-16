#include "app_main.hpp"
#include "comm.hpp"
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

TaskHandle_t motor_slow_loop_task_handle;
void motor_slow_loop(void* param);

// 存储配置数据
static Flash flash;

// 组装电机控制器
CommManager comm_manager(2);
Motor motor(flash, comm_manager);

UartComm uart_comm(BSP::uart_comm_port());

extern "C" void app_main(void* param) {
    int value1=0,value2=0;
    // bsp::hardware::spi1.init();
    // bsp::hardware::spi3.init();
    // bsp::hardware::i2c1.init();

    BSP::time_init();
    // BSP::adc_measure_start();
    // BSP::ic_spi_bus().init();
    //BSP::tim1_svpwm().init();
    // BSP::sensor_spi_bus().init();
    // BSP::sensor_i2c_bus().init();

    flash.init();
    // uart_comm.init();
    // uart_comm.register_recv_callback([](void* data,int size){
    //     bsp::hardware::led.write(true);
    // });

    // comm_manager.register_comm_channel(&uart_comm);
    // // comm_manager.register_comm_channel(&uart_comm); //其它的通信接口

    
    //flash.write_value("value1", 10);
    //flash.write_value("value2", 20);
    //flash.fresh();
    flash.read_value("value1", value1);
    flash.read_value("value2", value2);
    BSP::get_time_us();
    // bool ret = motor.init();
    bool ret=false;
    if (!ret) {
        while (1) {
            
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // BSP::tim1_svpwm().register_timer_update_cb([]() {
    //     motor.fast_loop();
    // });

    // xTaskCreate(motor_slow_loop, "motor_slow_loop", 256, NULL, 1, &motor_slow_loop_task_handle);
    
    // while (1) {
    //     motor.handle_event(); // 阻塞等待并处理电机事件
    // }
}

void motor_slow_loop(void* param) {
    while (1) {
        motor.slow_loop();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}