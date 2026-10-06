#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32g4xx_hal.h"

#include <cstdint>
#include <functional>

namespace bsp {

struct SPITransfer {
    const void* tx_data = nullptr;
    void* rx_data = nullptr;
    uint16_t size = 0U;
    void* param = nullptr;
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t error_code = HAL_SPI_ERROR_NONE;
    std::function<void(SPITransfer*)> pre_transmit_cb;
    std::function<void(SPITransfer*)> transmited_cb;

    struct TransferOptions {
        uint32_t data_size = SPI_DATASIZE_8BIT;
        uint32_t clk_polarity = SPI_POLARITY_LOW;
        uint32_t clk_phase = SPI_PHASE_1EDGE;
        uint32_t first_bit = SPI_FIRSTBIT_MSB;
    } options;
};

class SpiBus {
public:
    explicit SpiBus(SPI_HandleTypeDef& handle, uint32_t queue_length = 8U);
    ~SpiBus();

    SpiBus(const SpiBus&) = delete;
    SpiBus& operator=(const SpiBus&) = delete;

    bool init();
    bool transferIt(SPITransfer& transfer);
    bool transferDma(SPITransfer& transfer);
    bool abort();

    SPI_HandleTypeDef* native() const { return handle_; }
    bool valid() const { return handle_ != nullptr && queue_ != nullptr; }

private:
    enum class TransferMode : uint8_t {
        Interrupt,
        Dma,
    };

    struct QueuedTransfer {
        SPITransfer* transfer = nullptr;
        TransferMode mode = TransferMode::Interrupt;
    };

    static void txCpltCallback(SPI_HandleTypeDef* hspi);
    static void rxCpltCallback(SPI_HandleTypeDef* hspi);
    static void txRxCpltCallback(SPI_HandleTypeDef* hspi);
    static void txHalfCpltCallback(SPI_HandleTypeDef* hspi);
    static void rxHalfCpltCallback(SPI_HandleTypeDef* hspi);
    static void txRxHalfCpltCallback(SPI_HandleTypeDef* hspi);
    static void errorCallback(SPI_HandleTypeDef* hspi);
    static void abortCpltCallback(SPI_HandleTypeDef* hspi);

    static SpiBus* find(SPI_HandleTypeDef* hspi);

    bool inIsr() const;
    bool transferValid(const SPITransfer& transfer) const;
    HAL_StatusTypeDef startHalTransfer(SPITransfer& transfer, TransferMode mode) const;
    SPITransfer* clearActiveTransfer(bool from_isr);
    bool takeNextTransfer(bool from_isr, QueuedTransfer& queued, BaseType_t* higher_priority_task_woken);
    void completeTransfer(HAL_StatusTypeDef status, bool from_isr, bool start_next);
    bool startNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken = nullptr);
    bool enqueueTransfer(SPITransfer& transfer, TransferMode mode);

    SPI_HandleTypeDef* handle_ = nullptr;
    QueueHandle_t queue_ = nullptr;
    SPITransfer* active_transfer_ = nullptr;
    TransferMode active_mode_ = TransferMode::Interrupt;
};

} // namespace bsp

namespace BSP {
using SPIBus = bsp::SpiBus;
using SPITransfer = bsp::SPITransfer;
} // namespace BSP
