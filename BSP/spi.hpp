#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32g4xx_hal.h"

#include <cstddef>
#include <cstdint>

namespace bsp {

enum class SpiTransferMode : uint8_t {
    Interrupt,
    Dma,
};

enum class SpiTransferType : uint8_t {
    Transmit,
    Receive,
    TransmitReceive,
};

enum class SpiTransferStatus : uint8_t {
    Ok,
    Busy,
    Timeout,
    Error,
};

struct SpiHardwareParams {
    uint32_t data_size = 0U;
    uint32_t clk_polarity = 0U;
    uint32_t clk_phase = 0U;
    uint32_t first_bit = 0U;
};

struct SpiTransferEvent {
    using Callback = void (*)(SpiTransferEvent& event);

    SpiTransferMode mode = SpiTransferMode::Interrupt;
    SpiTransferType type = SpiTransferType::TransmitReceive;
    SpiHardwareParams hardware = {};
    const void* tx_data = nullptr;
    void* rx_data = nullptr;
    uint16_t size = 0U;
    void* context = nullptr;
    SpiTransferStatus status = SpiTransferStatus::Ok;
    Callback before_cb = nullptr;
    Callback after_cb = nullptr;
};

using SPITransfer = SpiTransferEvent;

class SpiBus {
public:
    explicit SpiBus(SPI_HandleTypeDef& handle, uint32_t queue_length = 8U);
    ~SpiBus();

    SpiBus(const SpiBus&) = delete;
    SpiBus& operator=(const SpiBus&) = delete;

    // Call from one task after scheduler startup, before transfers; keep the bus idle.
    bool init();
    bool transfer(SpiTransferEvent& event);
    bool abort();

    SPI_HandleTypeDef* native() const { return handle_; }
    bool valid() const { return handle_ != nullptr && queue_ != nullptr; }

private:
    static constexpr std::size_t kMaxBusCount = 4U;
    static SpiBus* buses_[kMaxBusCount];

    static void txCpltCallback(SPI_HandleTypeDef* hspi);
    static void rxCpltCallback(SPI_HandleTypeDef* hspi);
    static void txRxCpltCallback(SPI_HandleTypeDef* hspi);
    static void txHalfCpltCallback(SPI_HandleTypeDef* hspi);
    static void rxHalfCpltCallback(SPI_HandleTypeDef* hspi);
    static void txRxHalfCpltCallback(SPI_HandleTypeDef* hspi);
    static void errorCallback(SPI_HandleTypeDef* hspi);
    static void abortCpltCallback(SPI_HandleTypeDef* hspi);
    static SpiBus* find(SPI_HandleTypeDef* hspi);
    static void registerBus(SpiBus* bus);
    static void unregisterBus(SpiBus* bus);

    bool inIsr() const;
    bool transferValid(const SpiTransferEvent& event) const;
    bool enqueueTransfer(SpiTransferEvent& event);
    bool takeNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken);
    bool startNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken = nullptr);
    void completeTransfer(HAL_StatusTypeDef status, bool from_isr, bool start_next);
    void clearActiveTransfer(bool from_isr);
    HAL_StatusTypeDef startHalTransfer(SpiTransferEvent& event) const;
    HAL_StatusTypeDef applyHardwareParams(const SpiTransferEvent& event) const;

    SPI_HandleTypeDef* handle_ = nullptr;
    uint32_t queue_length_ = 0U;
    QueueHandle_t queue_ = nullptr;
    SpiTransferEvent active_event_ = {};
    bool active_ = false;
};

} // namespace bsp

namespace BSP {
using SPIBus = bsp::SpiBus;
using SPITransfer = bsp::SPITransfer;
using SpiHardwareParams = bsp::SpiHardwareParams;
using SpiTransferEvent = bsp::SpiTransferEvent;
using SpiTransferMode = bsp::SpiTransferMode;
using SpiTransferStatus = bsp::SpiTransferStatus;
using SpiTransferType = bsp::SpiTransferType;
} // namespace BSP
