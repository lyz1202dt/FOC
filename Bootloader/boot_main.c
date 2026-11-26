#include "boot_main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"

#include <stdbool.h>
#include <string.h>

#define FRAME_HEADER 0x5AU
#define FRAME_MIN_SIZE 8U
#define FRAME_MAX_SIZE 255U
#define APP_PAGE_COUNT (BOOT_APP_LENGTH / FLASH_PAGE_SIZE)

static UART_HandleTypeDef huart3;
static uint8_t erased_pages[(APP_PAGE_COUNT + 7U) / 8U];

static uint16_t FrameCrc(const uint8_t* data, uint32_t length) {
    uint16_t crc = 0xFFFFU;
    for (uint32_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint32_t bit = 0; bit < 8U; ++bit) {
            crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? 0xA001U : 0U);
        }
    }
    return crc;
}

static bool AppIsValid(void) {
    const uint32_t stack = *(const uint32_t*)BOOT_APP_ADDRESS;
    const uint32_t reset = *(const uint32_t*)(BOOT_APP_ADDRESS + 4U);
    return stack >= SRAM_BASE && stack <= SRAM_BASE + 0x8000U && (stack & 7U) == 0U &&
           (reset & 1U) != 0U && (reset & ~1U) >= BOOT_APP_ADDRESS &&
           (reset & ~1U) < BOOT_APP_ADDRESS + BOOT_APP_LENGTH;
}

static bool WriteApp(uint32_t offset, const uint8_t* data, uint32_t length) {
    if ((offset & 7U) != 0U || (length & 7U) != 0U ||
        offset >= BOOT_APP_LENGTH || length > BOOT_APP_LENGTH - offset) {
        return false;
    }
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }
    bool ok = true;
    for (uint32_t pos = 0; pos < length && ok; pos += 8U) {
        const uint32_t address = BOOT_APP_ADDRESS + offset + pos;
        const uint32_t page = (offset + pos) / FLASH_PAGE_SIZE;
        const uint8_t bit = (uint8_t)(1U << (page & 7U));
        if ((erased_pages[page / 8U] & bit) == 0U) {
            FLASH_EraseInitTypeDef erase = {0};
            uint32_t page_error = 0U;
            erase.TypeErase = FLASH_TYPEERASE_PAGES;
            erase.Banks = FLASH_BANK_1;
            erase.Page = (BOOT_APP_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE + page;
#if defined(FLASH_OPTR_DBANK)
            if ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0U && address >= FLASH_BASE + FLASH_BANK_SIZE) {
                erase.Banks = FLASH_BANK_2;
                erase.Page = (address - FLASH_BASE - FLASH_BANK_SIZE) / FLASH_PAGE_SIZE;
            }
#endif
            erase.NbPages = 1U;
            ok = HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK;
            if (ok) {
                erased_pages[page / 8U] |= bit;
            }
        }
        uint64_t word;
        memcpy(&word, data + pos, sizeof(word));
        if (ok && memcmp((const void*)address, &word, sizeof(word)) != 0) {
            if (*(const uint64_t*)address != UINT64_MAX) {
                ok = false;
            } else if (word != UINT64_MAX) {
                ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, word) == HAL_OK;
            }
        }
        if (ok) {
            ok = memcmp((const void*)address, &word, sizeof(word)) == 0;
        }
    }
    HAL_FLASH_Lock();
    return ok;
}

int main(void) {

    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    HAL_Delay(5); // 电平稳定
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) && AppIsValid()) {
        HAL_RCC_DeInit();
        HAL_DeInit();
        RunAPP();
    }
    UART_Init();
    uint8_t frame[FRAME_MAX_SIZE];
    for (;;) {
        if (HAL_UART_Receive(&huart3, frame, 1U, HAL_MAX_DELAY) != HAL_OK || frame[0] != FRAME_HEADER) {
            continue;
        }
        if (HAL_UART_Receive(&huart3, frame + 1U, 1U, 100U) != HAL_OK ||
            frame[1] < FRAME_MIN_SIZE) {
            continue;
        }
        const uint32_t size = frame[1];
        if (HAL_UART_Receive(&huart3, frame + 2U, size - 2U, 100U) != HAL_OK) {
            continue;
        }
        const uint16_t received_crc = (uint16_t)frame[size - 2U] | ((uint16_t)frame[size - 1U] << 8U);
        if (received_crc != FrameCrc(frame, size - 2U)) {
            continue;
        }
        const uint32_t offset = (uint32_t)frame[2] | ((uint32_t)frame[3] << 8U) |
                                ((uint32_t)frame[4] << 16U) | ((uint32_t)frame[5] << 24U);
        const uint32_t data_length = size - FRAME_MIN_SIZE;
        if (data_length == 0U ? !AppIsValid() : !WriteApp(offset, frame + 6U, data_length)) {
            continue;
        }
        uint8_t ack[FRAME_MIN_SIZE];
        memcpy(ack, frame, 6U);
        ack[1] = FRAME_MIN_SIZE;
        const uint16_t ack_crc = FrameCrc(ack, 6U);
        ack[6] = (uint8_t)ack_crc;
        ack[7] = (uint8_t)(ack_crc >> 8U);
        if (HAL_UART_Transmit(&huart3, ack, sizeof(ack), 100U) != HAL_OK) {
            continue;
        }
        if (data_length == 0U) {
            break;
        }
    }

    HAL_UART_DeInit(&huart3);
    HAL_RCC_DeInit();
    HAL_DeInit();
    RunAPP();
}

void RunAPP() {
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    for (uint32_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    SCB->VTOR = BOOT_APP_ADDRESS;
    __DSB();
    __ISB();
    BootJumpToApp(BOOT_APP_ADDRESS);
}


void UART_Init(void) {
    __HAL_RCC_USART3_CLK_ENABLE();
    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = 115200;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin              = GPIO_PIN_12;
    GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin       = GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin  = GPIO_PIN_9;  // BOOTLOADER选择引脚
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // 默认高电平，即默认跳转APP
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void SysTick_Handler(void) {
    HAL_IncTick();
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV5;
    RCC_OscInitStruct.PLL.PLLN       = 68;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}
