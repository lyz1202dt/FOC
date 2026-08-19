#include "app_main.hpp"
#include "hardware.hpp"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"

Motor motor;

void app_main(void* param) {
    BSP::time_init();
    bool success = motor.init();
    if (!success) {
        success = motor.init();
        if (!success) {
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
