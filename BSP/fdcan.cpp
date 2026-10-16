#include "fdcan.hpp"

#include "task.h"

namespace bsp {

FdcanBus* FdcanBus::buses_[FdcanBus::kMaxBusCount]{};

FdcanBus::FdcanBus(FDCAN_HandleTypeDef& handle, uint32_t queue_length)
    : handle_(&handle), queue_length_(queue_length)
{
}

FdcanBus::~FdcanBus()
{
    unregisterBus(this);
    if (tx_queue_ != nullptr) {
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
    }
}

void FdcanBus::registerBus(FdcanBus* bus)
{
    if (bus == nullptr) {
        return;
    }

    for (FdcanBus*& slot : buses_) {
        if (slot == bus) {
            return;
        }
        if (slot == nullptr) {
            slot = bus;
            return;
        }
    }
}

void FdcanBus::unregisterBus(FdcanBus* bus)
{
    for (FdcanBus*& slot : buses_) {
        if (slot == bus) {
            slot = nullptr;
        }
    }
}

FdcanBus* FdcanBus::find(FDCAN_HandleTypeDef* hfdcan)
{
    if (hfdcan == nullptr) {
        return nullptr;
    }

    for (FdcanBus* bus : buses_) {
        if (bus != nullptr && bus->handle_ == hfdcan) {
            return bus;
        }
    }
    return nullptr;
}

bool FdcanBus::inIsr() const
{
    return __get_IPSR() != 0U;
}

bool FdcanBus::init()
{
    // Initialize from one task after the scheduler starts, before using the bus.
    if (handle_ == nullptr || queue_length_ == 0U) {
        return false;
    }

    if (tx_queue_ == nullptr) {
        tx_queue_ = xQueueCreate(static_cast<UBaseType_t>(queue_length_), sizeof(FdcanTransferEvent));
    }
    if (tx_queue_ == nullptr) {
        return false;
    }

    registerBus(this);
    return true;
}

HAL_StatusTypeDef FdcanBus::configFilter(const FDCAN_FilterTypeDef& filter) const
{
    return HAL_FDCAN_ConfigFilter(handle_, &filter);
}

HAL_StatusTypeDef FdcanBus::configGlobalFilter(uint32_t non_matching_std,
                                               uint32_t non_matching_ext,
                                               uint32_t reject_remote_std,
                                               uint32_t reject_remote_ext) const
{
    return HAL_FDCAN_ConfigGlobalFilter(handle_,
                                        non_matching_std,
                                        non_matching_ext,
                                        reject_remote_std,
                                        reject_remote_ext);
}

HAL_StatusTypeDef FdcanBus::configFifoWatermark(uint32_t fifo, uint32_t watermark) const
{
    (void)fifo;
    (void)watermark;
    return HAL_OK;
}

HAL_StatusTypeDef FdcanBus::configAcceptAllFilters(uint32_t std_filter_index,
                                                   uint32_t ext_filter_index,
                                                   uint32_t fifo) const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    const uint32_t non_matching_fifo =
        fifo == FDCAN_FILTER_TO_RXFIFO1 || fifo == FDCAN_FILTER_TO_RXFIFO1_HP
            ? FDCAN_ACCEPT_IN_RX_FIFO1
            : FDCAN_ACCEPT_IN_RX_FIFO0;
    uint32_t non_matching_std = FDCAN_REJECT;
    uint32_t non_matching_ext = FDCAN_REJECT;

    if (handle_->Init.StdFiltersNbr > std_filter_index) {
        FDCAN_FilterTypeDef std_filter = {};
        std_filter.IdType = FDCAN_STANDARD_ID;
        std_filter.FilterIndex = std_filter_index;
        std_filter.FilterType = FDCAN_FILTER_MASK;
        std_filter.FilterID1 = 0x000U;
        std_filter.FilterID2 = 0x000U;
        std_filter.FilterConfig = fifo;

        const HAL_StatusTypeDef status = configFilter(std_filter);
        if (status != HAL_OK) {
            return status;
        }
    } else {
        non_matching_std = non_matching_fifo;
    }

    if (handle_->Init.ExtFiltersNbr > ext_filter_index) {
        FDCAN_FilterTypeDef ext_filter = {};
        ext_filter.IdType = FDCAN_EXTENDED_ID;
        ext_filter.FilterIndex = ext_filter_index;
        ext_filter.FilterType = FDCAN_FILTER_MASK;
        ext_filter.FilterID1 = 0x00000000U;
        ext_filter.FilterID2 = 0x00000000U;
        ext_filter.FilterConfig = fifo;

        const HAL_StatusTypeDef status = configFilter(ext_filter);
        if (status != HAL_OK) {
            return status;
        }
    } else {
        non_matching_ext = non_matching_fifo;
    }

    return configGlobalFilter(non_matching_std, non_matching_ext);
}

HAL_StatusTypeDef FdcanBus::start(uint32_t active_its, uint32_t rx_fifo0_watermark)
{
    if (!valid()) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = registerCallbacks();
    if (status != HAL_OK) {
        return status;
    }

    status = configFifoWatermark(FDCAN_RX_FIFO0, rx_fifo0_watermark);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_FDCAN_ActivateNotification(handle_, active_its, 0xFFFFFFFFU);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_FDCAN_Start(handle_);
}

HAL_StatusTypeDef FdcanBus::stop() const
{
    return HAL_FDCAN_Stop(handle_);
}

HAL_StatusTypeDef FdcanBus::activateNotification(uint32_t active_its,
                                                 uint32_t buffer_indexes) const
{
    return HAL_FDCAN_ActivateNotification(handle_, active_its, buffer_indexes);
}

HAL_StatusTypeDef FdcanBus::deactivateNotification(uint32_t inactive_its) const
{
    return HAL_FDCAN_DeactivateNotification(handle_, inactive_its);
}

bool FdcanBus::enqueueTransfer(FdcanTransferEvent& event)
{
    if (!valid() || event.frame.data == nullptr || event.size == 0U ||
        event.size > Frame::MaxDataLength || event.size > event.frame.data_capacity) {
        return false;
    }

    event.handle = handle_;
    event.status = HAL_OK;
    event.error_code = HAL_FDCAN_ERROR_NONE;
    event.tx_buffer_index = 0U;
    event.frame.data_length = lengthToDlc(event.size);

    if (inIsr()) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if (xQueueSendFromISR(tx_queue_, &event, &higher_priority_task_woken) != pdPASS) {
            return false;
        }
        (void)startNextTransfer(true, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
        return true;
    }

    if (xQueueSend(tx_queue_, &event, 0U) != pdPASS) {
        return false;
    }

    (void)startNextTransfer(false);
    return true;
}

bool FdcanBus::takeNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken)
{
    if (tx_active_) {
        return false;
    }

    bool has_transfer = false;
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        has_transfer =
            xQueueReceiveFromISR(tx_queue_, &active_tx_, higher_priority_task_woken) == pdPASS;
        tx_active_ = has_transfer;
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        has_transfer = xQueueReceive(tx_queue_, &active_tx_, 0U) == pdPASS;
        tx_active_ = has_transfer;
        taskEXIT_CRITICAL();
    }

    return has_transfer;
}

bool FdcanBus::startNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken)
{
    if (!takeNextTransfer(from_isr, higher_priority_task_woken)) {
        return false;
    }

    active_tx_.status = HAL_BUSY;
    if (active_tx_.before_cb != nullptr) {
        active_tx_.before_cb(active_tx_);
    }

    HAL_StatusTypeDef status =
        HAL_FDCAN_ActivateNotification(handle_, FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_ABORT_COMPLETE,
                                       0xFFFFFFFFU);
    if (status == HAL_OK) {
        FDCAN_TxHeaderTypeDef header = makeTxHeader(active_tx_.frame);
        status = HAL_FDCAN_AddMessageToTxFifoQ(handle_, &header, active_tx_.frame.data);
    }

    if (status != HAL_OK) {
        completeTransmit(status, from_isr, true);
        return false;
    }

    active_tx_.tx_buffer_index = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(handle_);
    if (active_tx_.tx_buffer_index == 0U) {
        completeTransmit(HAL_ERROR, from_isr, true);
        return false;
    }

    return true;
}

void FdcanBus::clearActiveTransfer(bool from_isr)
{
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        tx_active_ = false;
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        tx_active_ = false;
        taskEXIT_CRITICAL();
    }
}

void FdcanBus::completeTransmit(HAL_StatusTypeDef status, bool from_isr, bool start_next)
{
    if (tx_active_) {
        active_tx_.status = status;
        active_tx_.error_code = HAL_FDCAN_GetError(handle_);
        FdcanTransferEvent finished_event = active_tx_;
        clearActiveTransfer(from_isr);
        if (finished_event.after_cb != nullptr) {
            finished_event.after_cb(finished_event);
        }
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (start_next) {
        (void)startNextTransfer(from_isr, &higher_priority_task_woken);
    }
    if (from_isr) {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

HAL_StatusTypeDef FdcanBus::transmit(FdcanTransferEvent& event)
{
    return enqueueTransfer(event) ? HAL_OK : HAL_BUSY;
}

HAL_StatusTypeDef FdcanBus::register_recv_cb(std::function<void(const Frame&)> recv_cb,
                                             uint32_t rx_location)
{
    if (!recv_cb || (rx_location != FDCAN_RX_FIFO0 && rx_location != FDCAN_RX_FIFO1)) {
        return HAL_ERROR;
    }

    registerBus(this);
    for (ReceiveCallbackSlot& slot : receive_callbacks_) {
        if (!slot.used) {
            slot.used = true;
            slot.rx_location = rx_location;
            slot.callback = std::move(recv_cb);
            return HAL_OK;
        }
    }

    return HAL_BUSY;
}

HAL_StatusTypeDef FdcanBus::receiveFrame(Frame& frame, uint32_t rx_location)
{
    FDCAN_RxHeaderTypeDef header = {};
    frame.data = rx_data_;
    frame.data_capacity = Frame::MaxDataLength;
    const HAL_StatusTypeDef status =
        HAL_FDCAN_GetRxMessage(handle_, rx_location, &header, frame.data);
    if (status != HAL_OK) {
        return status;
    }

    frame.id = header.Identifier;
    frame.id_type = header.IdType;
    frame.frame_type = header.RxFrameType;
    frame.data_length = header.DataLength;
    frame.error_state_indicator = header.ErrorStateIndicator;
    frame.bitrate_switch = header.BitRateSwitch;
    frame.fd_format = header.FDFormat;
    return HAL_OK;
}

void FdcanBus::dispatchReceive(uint32_t rx_location)
{
    bool has_callback = false;
    for (const ReceiveCallbackSlot& slot : receive_callbacks_) {
        if (slot.used && slot.rx_location == rx_location && slot.callback) {
            has_callback = true;
            break;
        }
    }
    if (!has_callback) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(handle_, rx_location) != 0U) {
        Frame frame = {};
        if (receiveFrame(frame, rx_location) != HAL_OK) {
            return;
        }

        for (const ReceiveCallbackSlot& slot : receive_callbacks_) {
            if (slot.used && slot.rx_location == rx_location && slot.callback) {
                slot.callback(frame);
            }
        }
    }
}

uint32_t FdcanBus::lengthToDlc(uint8_t length)
{
    if (length <= 8U) {
        return length;
    }
    if (length <= 12U) {
        return FDCAN_DLC_BYTES_12;
    }
    if (length <= 16U) {
        return FDCAN_DLC_BYTES_16;
    }
    if (length <= 20U) {
        return FDCAN_DLC_BYTES_20;
    }
    if (length <= 24U) {
        return FDCAN_DLC_BYTES_24;
    }
    if (length <= 32U) {
        return FDCAN_DLC_BYTES_32;
    }
    if (length <= 48U) {
        return FDCAN_DLC_BYTES_48;
    }
    return FDCAN_DLC_BYTES_64;
}

uint8_t FdcanBus::dlcToLength(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0:
        return 0U;
    case FDCAN_DLC_BYTES_1:
        return 1U;
    case FDCAN_DLC_BYTES_2:
        return 2U;
    case FDCAN_DLC_BYTES_3:
        return 3U;
    case FDCAN_DLC_BYTES_4:
        return 4U;
    case FDCAN_DLC_BYTES_5:
        return 5U;
    case FDCAN_DLC_BYTES_6:
        return 6U;
    case FDCAN_DLC_BYTES_7:
        return 7U;
    case FDCAN_DLC_BYTES_8:
        return 8U;
    case FDCAN_DLC_BYTES_12:
        return 12U;
    case FDCAN_DLC_BYTES_16:
        return 16U;
    case FDCAN_DLC_BYTES_20:
        return 20U;
    case FDCAN_DLC_BYTES_24:
        return 24U;
    case FDCAN_DLC_BYTES_32:
        return 32U;
    case FDCAN_DLC_BYTES_48:
        return 48U;
    case FDCAN_DLC_BYTES_64:
        return 64U;
    default:
        return 0U;
    }
}

FDCAN_TxHeaderTypeDef FdcanBus::makeTxHeader(const Frame& frame)
{
    FDCAN_TxHeaderTypeDef header = {};
    header.Identifier = frame.id;
    header.IdType = frame.id_type;
    header.TxFrameType = frame.frame_type;
    header.DataLength = frame.data_length;
    header.ErrorStateIndicator = frame.error_state_indicator;
    header.BitRateSwitch = frame.bitrate_switch;
    header.FDFormat = frame.fd_format;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0x00U;
    return header;
}

HAL_StatusTypeDef FdcanBus::registerCallbacks()
{
#if (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
    bool registered = true;

    if (HAL_FDCAN_RegisterTxEventFifoCallback(
            handle_, &FdcanBus::txEventFifoCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterRxFifo0Callback(
            handle_, &FdcanBus::rxFifo0Callback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterRxFifo1Callback(
            handle_, &FdcanBus::rxFifo1Callback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterCallback(
            handle_, HAL_FDCAN_TX_FIFO_EMPTY_CB_ID, &FdcanBus::txFifoEmptyCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterTxBufferCompleteCallback(
            handle_, &FdcanBus::txBufferCompleteCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterTxBufferAbortCallback(
            handle_, &FdcanBus::txBufferAbortCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterCallback(
            handle_, HAL_FDCAN_HIGH_PRIO_MESSAGE_CB_ID,
            &FdcanBus::highPriorityMessageCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterCallback(
            handle_, HAL_FDCAN_TIMESTAMP_WRAPAROUND_CB_ID,
            &FdcanBus::timestampWraparoundCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterCallback(
            handle_, HAL_FDCAN_TIMEOUT_OCCURRED_CB_ID,
            &FdcanBus::timeoutOccurredCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterCallback(
            handle_, HAL_FDCAN_ERROR_CALLBACK_CB_ID, &FdcanBus::errorCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterErrorStatusCallback(
            handle_, &FdcanBus::errorStatusCallback) != HAL_OK) {
        registered = false;
    }
    if (registered) {
        registerBus(this);
    }
    return registered ? HAL_OK : HAL_ERROR;
#else
    return HAL_ERROR;
#endif
}

void FdcanBus::txEventFifoCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t tx_event_fifo_its)
{
    (void)hfdcan;
    (void)tx_event_fifo_its;
}

void FdcanBus::rxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its)
{
    (void)rx_fifo0_its;
    FdcanBus* bus = find(hfdcan);
    if (bus != nullptr) {
        bus->dispatchReceive(FDCAN_RX_FIFO0);
    }
}

void FdcanBus::rxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo1_its)
{
    (void)rx_fifo1_its;
    FdcanBus* bus = find(hfdcan);
    if (bus != nullptr) {
        bus->dispatchReceive(FDCAN_RX_FIFO1);
    }
}

void FdcanBus::txFifoEmptyCallback(FDCAN_HandleTypeDef* hfdcan)
{
    (void)hfdcan;
}

void FdcanBus::txBufferCompleteCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t buffer_indexes)
{
    FdcanBus* bus = find(hfdcan);
    if (bus != nullptr && bus->tx_active_ &&
        (buffer_indexes & bus->active_tx_.tx_buffer_index) != 0U) {
        bus->completeTransmit(HAL_OK, true, true);
    }
}

void FdcanBus::txBufferAbortCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t buffer_indexes)
{
    FdcanBus* bus = find(hfdcan);
    if (bus != nullptr && bus->tx_active_ &&
        (buffer_indexes & bus->active_tx_.tx_buffer_index) != 0U) {
        bus->completeTransmit(HAL_ERROR, true, true);
    }
}

void FdcanBus::highPriorityMessageCallback(FDCAN_HandleTypeDef* hfdcan)
{
    (void)hfdcan;
}

void FdcanBus::timestampWraparoundCallback(FDCAN_HandleTypeDef* hfdcan)
{
    (void)hfdcan;
}

void FdcanBus::timeoutOccurredCallback(FDCAN_HandleTypeDef* hfdcan)
{
    FdcanBus* bus = find(hfdcan);
    if (bus != nullptr) {
        bus->completeTransmit(HAL_ERROR, true, true);
    }
}

void FdcanBus::errorCallback(FDCAN_HandleTypeDef* hfdcan)
{
    FdcanBus* bus = find(hfdcan);
    if (bus != nullptr) {
        bus->completeTransmit(HAL_ERROR, true, true);
    }
}

void FdcanBus::errorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t error_status_its)
{
    (void)error_status_its;
    FdcanBus* bus = find(hfdcan);
    if (bus != nullptr) {
        bus->completeTransmit(HAL_ERROR, true, true);
    }
}

} // namespace bsp
