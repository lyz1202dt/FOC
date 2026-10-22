# Build HAL separately so the bootloader keeps its own vector table offset.
set(BOOT_HAL_DIR ${PROJECT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver)
add_executable(bootloader
    boot_main.c
    boot_jump.s
    system_stm32g4xx.c
    ${PROJECT_SOURCE_DIR}/startup_stm32g431xx.s)
target_sources(bootloader PRIVATE
    ${BOOT_HAL_DIR}/Src/stm32g4xx_hal.c
    ${BOOT_HAL_DIR}/Src/stm32g4xx_hal_cortex.c
    ${BOOT_HAL_DIR}/Src/stm32g4xx_hal_rcc.c
    ${BOOT_HAL_DIR}/Src/stm32g4xx_hal_rcc_ex.c
    ${BOOT_HAL_DIR}/Src/stm32g4xx_hal_gpio.c
    ${BOOT_HAL_DIR}/Src/stm32g4xx_hal_pwr.c
    ${BOOT_HAL_DIR}/Src/stm32g4xx_hal_pwr_ex.c
    ${BOOT_HAL_DIR}/Src/stm32g4xx_hal_dma.c
    ${BOOT_HAL_DIR}/Src/stm32g4xx_hal_uart.c
    ${BOOT_HAL_DIR}/Src/stm32g4xx_hal_uart_ex.c)

target_include_directories(bootloader PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/Core/Inc
    ${BOOT_HAL_DIR}/Inc
    ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/Include)
target_compile_definitions(bootloader PRIVATE
    USE_HAL_DRIVER STM32G431xx USER_VECT_TAB_ADDRESS VECT_TAB_OFFSET=0
    BOOT_APP_ADDRESS=${APP_FLASH_ORIGIN})
configure_firmware(bootloader "${CMAKE_CURRENT_SOURCE_DIR}/STM32G431XX_FLASH.ld"
    ${BOOT_FLASH_ORIGIN} ${BOOT_FLASH_LENGTH})
