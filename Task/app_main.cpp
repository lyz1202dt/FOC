#include "app_main.hpp"
#include "i2c.hpp"
#include "spi.hpp"
#include "hardware.hpp"
#include "measure.hpp"
#include "pack.h"
#include "svpwm.hpp"

#include "as5600.hpp"
#include "drv8301.hpp"
#include "estimater.hpp"
#include "flash.hpp"
#include "foc.hpp"
#include "motor.hpp"
#include "parser.hpp"

static Flash flash;
static AS5600 encoder(BSP::sensor_i2c_bus());
static FOC controller;
static Estimater estimater;
static DRV8301 driver(
    BSP::ic_spi_bus(), BSP::tim1_svpwm(), BSP::adc_measure(), BSP::drv8301_enable_gate(), BSP::drv8301_fault(), BSP::drv8301_dc_cal());
Motor motor(&flash, &encoder, &driver, &controller, &estimater);
PackParser parser(&BSP::uart_comm_port());

void on_pwm_update(void* context) {
    static_cast<Motor*>(context)->fast_loop();
}

void on_target_update(const Command* cmd) {
    MotorCommand* temp = ((MotorCommand*)cmd->data);
    motor.controller_->set_controller_reference(temp->position, temp->velocity, temp->torque, temp->kp, temp->kd);
}

void on_enable_motor(const Command* cmd)
{
    motor.enable(true);
}

extern "C" void app_main(void* param) {
    BSP::time_init();
    BSP::adc_measure().init();
    BSP::ic_spi_bus().init();
    BSP::tim1_svpwm().init();
    BSP::sensor_spi_bus().init();
    BSP::sensor_i2c_bus().init();
    BSP::uart_comm_port().init();

    parser.register_cmd_callback(0x01, on_enable_motor);  //电机使能
    parser.register_cmd_callback(0x02, on_target_update); //电机期望值更新
    

    bool success = motor.init();
    if (!success) {
        success = motor.init();
        if (!success) {
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
    BSP::tim1_svpwm().register_timer_update_cb(on_pwm_update, &motor);
    while (1) {
        motor.state_update();                          // 状态更新循环
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
