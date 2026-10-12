#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32g4xx_hal.h"

#include <cstddef>
#include <cstdint>
#include <functional>

namespace bsp {

struct FdcanFrame {
    static constexpr std::size_t MaxDataLength = 64U;

    uint32_t id = 0U;
    uint32_t id_type = FDCAN_STANDARD_ID;
    uint32_t frame_type = FDCAN_DATA_FRAME;
    uint32_t data_length = FDCAN_DLC_BYTES_8;
    uint32_t error_state_indicator = FDCAN_ESI_ACTIVE;
    uint32_t bitrate_switch = FDCAN_BRS_OFF;
    uint32_t fd_format = FDCAN_CLASSIC_CAN;
    uint8_t* data = nullptr;
    std::size_t data_capacity = 0U;
};

struct FdcanTransferEvent {
    using Callback = void (*)(FdcanTransferEvent& event);

    FDCAN_HandleTypeDef* handle = nullptr;
    FdcanFrame frame = {};
    uint8_t size = 0U;
    uint32_t tx_buffer_index = 0U;
    void* context = nullptr;
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t error_code = HAL_FDCAN_ERROR_NONE;
    Callback before_cb = nullptr;
    Callback after_cb = nullptr;
};

class FdcanBus {
public:
    using Frame = FdcanFrame;

    explicit FdcanBus(FDCAN_HandleTypeDef& handle, uint32_t queue_length = 8U);
    ~FdcanBus();

    FdcanBus(const FdcanBus&) = delete;
    FdcanBus& operator=(const FdcanBus&) = delete;

    HAL_StatusTypeDef configFilter(const FDCAN_FilterTypeDef& filter) const;
    HAL_StatusTypeDef configGlobalFilter(uint32_t non_matching_std = FDCAN_REJECT,
                                         uint32_t non_matching_ext = FDCAN_REJECT,
                                         uint32_t reject_remote_std = FDCAN_FILTER_REMOTE,
                                         uint32_t reject_remote_ext = FDCAN_FILTER_REMOTE) const;
    HAL_StatusTypeDef configFifoWatermark(uint32_t fifo = FDCAN_RX_FIFO0,
                                          uint32_t watermark = 1U) const;
    HAL_StatusTypeDef configAcceptAllFilters(uint32_t std_filter_index = 0U,
                                             uint32_t ext_filter_index = 1U,
                                             uint32_t fifo = FDCAN_FILTER_TO_RXFIFO0) const;

    HAL_StatusTypeDef start(uint32_t active_its = FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                            uint32_t rx_fifo0_watermark = 1U);
    HAL_StatusTypeDef stop() const;

    HAL_StatusTypeDef activateNotification(uint32_t active_its,
                                           uint32_t buffer_indexes = 0U) const;
    HAL_StatusTypeDef deactivateNotification(uint32_t inactive_its) const;

    HAL_StatusTypeDef transmit(FdcanTransferEvent& event);
    HAL_StatusTypeDef register_recv_cb(std::function<void(const Frame& frame)> recv_cb,
                                       uint32_t rx_location = FDCAN_RX_FIFO0);

    FDCAN_HandleTypeDef* native() const { return handle_; }
    bool valid() const { return handle_ != nullptr && tx_queue_ != nullptr; }

    static uint32_t lengthToDlc(uint8_t length);
    static uint8_t dlcToLength(uint32_t dlc);

private:
    struct ReceiveCallbackSlot {
        bool used = false;
        uint32_t rx_location = FDCAN_RX_FIFO0;
        std::function<void(const Frame&)> callback;
    };

    static constexpr std::size_t kMaxBusCount = 4U;
    static constexpr std::size_t kMaxReceiveCallbackCount = 4U;
    static FdcanBus* buses_[kMaxBusCount];

    static FDCAN_TxHeaderTypeDef makeTxHeader(const Frame& frame);
    static FdcanBus* find(FDCAN_HandleTypeDef* hfdcan);
    static void registerBus(FdcanBus* bus);
    static void unregisterBus(FdcanBus* bus);

    HAL_StatusTypeDef registerCallbacks();
    HAL_StatusTypeDef receiveFrame(Frame& frame, uint32_t rx_location);
    void dispatchReceive(uint32_t rx_location);
    bool inIsr() const;
    bool enqueueTransfer(FdcanTransferEvent& event);
    bool takeNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken);
    bool startNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken = nullptr);
    void clearActiveTransfer(bool from_isr);
    void completeTransmit(HAL_StatusTypeDef status, bool from_isr, bool start_next);

    static void txEventFifoCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t tx_event_fifo_its);
    static void rxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its);
    static void rxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo1_its);
    static void txFifoEmptyCallback(FDCAN_HandleTypeDef* hfdcan);
    static void txBufferCompleteCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t buffer_indexes);
    static void txBufferAbortCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t buffer_indexes);
    static void highPriorityMessageCallback(FDCAN_HandleTypeDef* hfdcan);
    static void timestampWraparoundCallback(FDCAN_HandleTypeDef* hfdcan);
    static void timeoutOccurredCallback(FDCAN_HandleTypeDef* hfdcan);
    static void errorCallback(FDCAN_HandleTypeDef* hfdcan);
    static void errorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t error_status_its);

    FDCAN_HandleTypeDef* handle_ = nullptr;
    QueueHandle_t tx_queue_ = nullptr;
    FdcanTransferEvent active_tx_ = {};
    uint8_t rx_data_[Frame::MaxDataLength] = {};
    bool tx_active_ = false;
    ReceiveCallbackSlot receive_callbacks_[kMaxReceiveCallbackCount]{};
};

} // namespace bsp

namespace BSP {
using FdcanBus = bsp::FdcanBus;
using FdcanFrame = bsp::FdcanFrame;
using FdcanTransferEvent = bsp::FdcanTransferEvent;
} // namespace BSP
