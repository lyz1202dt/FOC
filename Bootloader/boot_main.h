#ifndef BOOTLOADER_BOOT_MAIN_H_
#define BOOTLOADER_BOOT_MAIN_H_

#include "stm32g4xx_hal.h"

void UART_Init(void);
void GPIO_Init(void);
void SystemClock_Config(void);
void Error_Handler(void);
void RunAPP();


__attribute__((noreturn)) void BootJumpToApp(uint32_t address);

#endif  // BOOTLOADER_BOOT_MAIN_H_
