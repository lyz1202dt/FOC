#include "app_main.hpp"
#include "hardware.hpp"
#include "tim.h"

Motor motor;

extern "C" {
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
}