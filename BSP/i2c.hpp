#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32g4xx_hal.h"

#include <cstddef>
#include <cstdint>
#include <functional>

namespace bsp {

enum class I2COperation : uint8_t {
    MasterTransmit,
    MasterReceive,
    MemoryWrite,
    MemoryRead,
};

struct I2CTransfer {
    I2COperation operation = I2COperation::MasterTransmit;
    uint16_t device_address = 0U;
    uint16_t memory_address = 0U;
    uint16_t memory_address_size = I2C_MEMADD_SIZE_8BIT;
    const void* tx_data = nullptr;
    void* rx_data = nullptr;
    uint16_t size = 0U;
    void* param = nullptr;
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t error_code = HAL_I2C_ERROR_NONE;
    std::function<void(I2CTransfer*)> transmited_cb;
};

class I2CBus {
public:
    explicit I2CBus(I2C_HandleTypeDef& handle, uint32_t queue_length = 8U);
    ~I2CBus();

    I2CBus(const I2CBus&) = delete;
    I2CBus& operator=(const I2CBus&) = delete;

    // Call from one task after scheduler startup, before transfers; keep the bus idle.
    bool init();
    bool transferIt(I2CTransfer& transfer);
    bool abort();

    I2C_HandleTypeDef* native() const { return handle_; }
    bool valid() const { return handle_ != nullptr && queue_ != nullptr; }

private:
    struct QueuedTransfer {
        I2CTransfer* transfer = nullptr;
    };

    static constexpr std::size_t kMaxBusCount = 4U;
    static I2CBus* buses_[kMaxBusCount];

    static void masterTxCpltCallback(I2C_HandleTypeDef* hi2c);
    static void masterRxCpltCallback(I2C_HandleTypeDef* hi2c);
    static void memTxCpltCallback(I2C_HandleTypeDef* hi2c);
    static void memRxCpltCallback(I2C_HandleTypeDef* hi2c);
    static void errorCallback(I2C_HandleTypeDef* hi2c);
    static void abortCpltCallback(I2C_HandleTypeDef* hi2c);

    static I2CBus* find(I2C_HandleTypeDef* hi2c);
    static void registerBus(I2CBus* bus);
    static void unregisterBus(I2CBus* bus);

    bool inIsr() const;
    bool transferValid(const I2CTransfer& transfer) const;
    HAL_StatusTypeDef startHalTransfer(I2CTransfer& transfer) const;
    I2CTransfer* clearActiveTransfer(bool from_isr);
    bool takeNextTransfer(bool from_isr, QueuedTransfer& queued, BaseType_t* higher_priority_task_woken);
    void completeTransfer(HAL_StatusTypeDef status, bool from_isr, bool start_next);
    bool startNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken = nullptr);
    bool enqueueTransfer(I2CTransfer& transfer);

    I2C_HandleTypeDef* handle_ = nullptr;
    uint32_t queue_length_ = 0U;
    QueueHandle_t queue_ = nullptr;
    I2CTransfer* active_transfer_ = nullptr;
};

} // namespace bsp

namespace BSP {
using I2CBus = bsp::I2CBus;
using I2COperation = bsp::I2COperation;
using I2CTransfer = bsp::I2CTransfer;
} // namespace BSP
