#include "drv8301.hpp"

#include "main.h"

DRV8301::DRV8301() {}

bool DRV8301::init() {
    enable();
    return true;
}

bool DRV8301::is_ready() {
    return HAL_GPIO_ReadPin(IC_FAULT_GPIO_Port, IC_FAULT_Pin) == GPIO_PIN_SET;
}

DRV8301::FaultType_e DRV8301::get_error() {
    return is_ready() ? FaultType_NoFault : FaultType_FAULT;
}

void DRV8301::enable() {
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);
}

void DRV8301::disable() {
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
}
