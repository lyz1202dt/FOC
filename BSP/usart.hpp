#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32g4xx_hal.h"

#include <cstddef>
#include <cstdint>

namespace bsp {

enum class Status: uint8_t{
    OK       = 0x00U,
    ERROR    = 0x01U,
    BUSY     = 0x02U
};

enum class UartTransmitMode : uint8_t {
    Interrupt,
    Dma,
};

enum class UartReceiveMode : uint8_t {
    Interrupt,
    Dma,
    ToIdleDma,
};

struct UartTransmitEvent {
    using Callback = void (*)(UartTransmitEvent& event);

    UartTransmitMode mode = UartTransmitMode::Interrupt;
    const void* data = nullptr;
    uint16_t size = 0U;
    void* context = nullptr;
    Status status = Status::OK;
    uint32_t error_code = HAL_UART_ERROR_NONE;
    Callback before_cb = nullptr;
    Callback after_cb = nullptr;
    bool queue_front = false;
};

struct UartReceiveEvent {
    using Callback = void (*)(UartReceiveEvent& event);

    UartReceiveMode mode = UartReceiveMode::Interrupt;
    void* data = nullptr;
    uint16_t size = 0U;
    uint16_t transferred_size = 0U;
    void* context = nullptr;
    Status status = Status::OK;
    uint32_t error_code = HAL_UART_ERROR_NONE;
    Callback before_cb = nullptr;
    Callback after_cb = nullptr;
};

class UartPort {
public:
    explicit UartPort(UART_HandleTypeDef& handle, uint32_t queue_length = 8U);
    ~UartPort();

    UartPort(const UartPort&) = delete;
    UartPort& operator=(const UartPort&) = delete;

    // Call from one task after scheduler startup, before transfers; keep the bus idle.
    bool init();
    // Queues descriptors only: data must remain valid/unchanged until after_cb.
    // OK means accepted, not completed; BUSY means queue full.
    Status transmit(UartTransmitEvent& event);
    Status receive(UartReceiveEvent& event);

    UART_HandleTypeDef* native() const { return handle_; }

private:
    static constexpr std::size_t kMaxPortCount = 4U;
    static UartPort* ports_[kMaxPortCount];

    static void txCpltCallback(UART_HandleTypeDef* huart);
    static void rxCpltCallback(UART_HandleTypeDef* huart);
    static void errorCallback(UART_HandleTypeDef* huart);
    static void rxEventCallback(UART_HandleTypeDef* huart, uint16_t size);

    static UartPort* find(UART_HandleTypeDef* huart);
    static void registerPort(UartPort* port);
    static void unregisterPort(UartPort* port);

    bool startNextTransmit(bool from_isr, BaseType_t* higher_priority_task_woken = nullptr);
    bool startNextReceive(bool from_isr, BaseType_t* higher_priority_task_woken = nullptr);
    void completeTransmit(HAL_StatusTypeDef status, bool from_isr, bool start_next,
                          uint32_t error_code = HAL_UART_ERROR_NONE);
    void completeReceive(HAL_StatusTypeDef status,
                         uint16_t transferred_size,
                         bool from_isr,
                         bool start_next,
                         uint32_t error_code = HAL_UART_ERROR_NONE);

    UART_HandleTypeDef* handle_ = nullptr;
    uint32_t queue_length_ = 0U;
    QueueHandle_t tx_queue_ = nullptr;
    QueueHandle_t rx_queue_ = nullptr;
    UartTransmitEvent active_tx_ = {};
    UartReceiveEvent active_rx_ = {};
    bool tx_active_ = false;
    bool rx_active_ = false;
};

} // namespace bsp

namespace BSP {
using UartPort = bsp::UartPort;
using UartReceiveEvent = bsp::UartReceiveEvent;
using UartReceiveMode = bsp::UartReceiveMode;
using UartTransmitEvent = bsp::UartTransmitEvent;
using UartTransmitMode = bsp::UartTransmitMode;
} // namespace BSP
