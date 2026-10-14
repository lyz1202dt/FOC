#include "app_main.hpp"
#include "comm.hpp"
#include "i2c.hpp"
#include "spi.hpp"
#include "hardware.hpp"
#include "pack.h"
#include "svpwm.hpp"


#include "flash.hpp"
#include "motor.hpp"
#include "comm.hpp"
#include "uartcomm.hpp"

//存储配置数据
static Flash flash;

//组装电机控制器
CommManager comm_manager(2);
Motor motor(flash,comm_manager);

UartComm uart_comm(BSP::uart_comm_port());

extern "C" void app_main(void* param) {
    BSP::time_init();
    BSP::adc_measure_start();
    BSP::ic_spi_bus().init();
    BSP::tim1_svpwm().init();
    BSP::sensor_spi_bus().init();
    BSP::sensor_i2c_bus().init();

    uart_comm.init();

    comm_manager.register_comm_channel(&uart_comm);
    //comm_manager.register_comm_channel(&uart_comm); //其它的通信接口

    BSP::tim1_svpwm().register_timer_update_cb([]() {
        motor.fast_loop();
    });
    while (1) {
        motor.state_update();                          // 状态更新循环（优先级最低-1kHz）
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
