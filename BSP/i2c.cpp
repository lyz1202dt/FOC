#include "i2c.hpp"

#include "task.h"

namespace bsp {

I2CBus* I2CBus::buses_[I2CBus::kMaxBusCount]{};

void I2CBus::registerBus(I2CBus* bus)
{
    if (bus == nullptr) {
        return;
    }

    for (I2CBus*& slot : buses_) {
        if (slot == bus) {
            return;
        }
        if (slot == nullptr) {
            slot = bus;
            return;
        }
    }
}

void I2CBus::unregisterBus(I2CBus* bus)
{
    for (I2CBus*& slot : buses_) {
        if (slot == bus) {
            slot = nullptr;
        }
    }
}

I2CBus::I2CBus(I2C_HandleTypeDef& handle, uint32_t queue_length)
    : handle_(&handle), queue_length_(queue_length)
{
}

I2CBus::~I2CBus()
{
    unregisterBus(this);
    if (queue_ != nullptr) {
        vQueueDelete(queue_);
        queue_ = nullptr;
    }
}

I2CBus* I2CBus::find(I2C_HandleTypeDef* hi2c)
{
    if (hi2c == nullptr) {
        return nullptr;
    }

    for (I2CBus* bus : buses_) {
        if (bus != nullptr && bus->handle_ == hi2c) {
            return bus;
        }
    }
    return nullptr;
}

bool I2CBus::inIsr() const
{
    return __get_IPSR() != 0U;
}

bool I2CBus::transferValid(const I2CTransfer& transfer) const
{
    if (transfer.size == 0U) {
        return false;
    }

    switch (transfer.operation) {
    case I2COperation::MasterTransmit:
    case I2COperation::MemoryWrite:
        return transfer.tx_data != nullptr;
    case I2COperation::MasterReceive:
    case I2COperation::MemoryRead:
        return transfer.rx_data != nullptr;
    default:
        return false;
    }
}

HAL_StatusTypeDef I2CBus::startHalTransfer(I2CTransfer& transfer) const
{
    auto* tx_data = const_cast<uint8_t*>(static_cast<const uint8_t*>(transfer.tx_data));
    auto* rx_data = static_cast<uint8_t*>(transfer.rx_data);

    switch (transfer.operation) {
    case I2COperation::MasterTransmit:
        return HAL_I2C_Master_Transmit_IT(handle_, transfer.device_address, tx_data, transfer.size);
    case I2COperation::MasterReceive:
        return HAL_I2C_Master_Receive_IT(handle_, transfer.device_address, rx_data, transfer.size);
    case I2COperation::MemoryWrite:
        return HAL_I2C_Mem_Write_IT(handle_, transfer.device_address, transfer.memory_address,
                                    transfer.memory_address_size, tx_data, transfer.size);
    case I2COperation::MemoryRead:
        return HAL_I2C_Mem_Read_IT(handle_, transfer.device_address, transfer.memory_address,
                                   transfer.memory_address_size, rx_data, transfer.size);
    default:
        return HAL_ERROR;
    }
}

I2CTransfer* I2CBus::clearActiveTransfer(bool from_isr)
{
    I2CTransfer* transfer = nullptr;
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        transfer = active_transfer_;
        active_transfer_ = nullptr;
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        transfer = active_transfer_;
        active_transfer_ = nullptr;
        taskEXIT_CRITICAL();
    }

    return transfer;
}

bool I2CBus::takeNextTransfer(bool from_isr, QueuedTransfer& queued, BaseType_t* higher_priority_task_woken)
{
    if (queue_ == nullptr) {
        return false;
    }

    bool has_transfer = false;
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        if (active_transfer_ == nullptr && xQueueReceiveFromISR(queue_, &queued, higher_priority_task_woken) == pdPASS) {
            active_transfer_ = queued.transfer;
            has_transfer = true;
        }
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        if (active_transfer_ == nullptr && xQueueReceive(queue_, &queued, 0U) == pdPASS) {
            active_transfer_ = queued.transfer;
            has_transfer = true;
        }
        taskEXIT_CRITICAL();
    }

    return has_transfer;
}

bool I2CBus::startNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken)
{
    QueuedTransfer queued;
    if (!takeNextTransfer(from_isr, queued, higher_priority_task_woken)) {
        return false;
    }

    I2CTransfer* transfer = queued.transfer;
    if (transfer == nullptr || !transferValid(*transfer)) {
        completeTransfer(HAL_ERROR, from_isr, true);
        return false;
    }

    transfer->status = HAL_BUSY;
    transfer->error_code = HAL_I2C_ERROR_NONE;

    const HAL_StatusTypeDef status = startHalTransfer(*transfer);
    if (status != HAL_OK) {
        completeTransfer(status, from_isr, true);
        return false;
    }

    return true;
}

void I2CBus::completeTransfer(HAL_StatusTypeDef status, bool from_isr, bool start_next)
{
    I2CTransfer* transfer = clearActiveTransfer(from_isr);
    if (transfer != nullptr) {
        transfer->status = status;
        transfer->error_code = HAL_I2C_GetError(handle_);
        if (transfer->transmited_cb) {
            transfer->transmited_cb(transfer);
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

bool I2CBus::enqueueTransfer(I2CTransfer& transfer)
{
    if (!valid() || !transferValid(transfer)) {
        return false;
    }

    const QueuedTransfer queued{&transfer};
    if (inIsr()) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if (xQueueSendFromISR(queue_, &queued, &higher_priority_task_woken) != pdPASS) {
            return false;
        }

        (void)startNextTransfer(true, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
        return true;
    }

    if (xQueueSend(queue_, &queued, 0U) != pdPASS) {
        return false;
    }

    (void)startNextTransfer(false);
    return true;
}

bool I2CBus::init()
{
    // Initialize from one task after the scheduler starts, before using the bus.
    if (handle_ == nullptr || queue_length_ == 0U) {
        return false;
    }

    if (queue_ == nullptr) {
        queue_ = xQueueCreate(static_cast<UBaseType_t>(queue_length_), sizeof(QueuedTransfer));
    }
    if (queue_ == nullptr) {
        return false;
    }

    active_transfer_ = nullptr;
    xQueueReset(queue_);

    bool registered = true;
    if (HAL_I2C_RegisterCallback(handle_, HAL_I2C_MASTER_TX_COMPLETE_CB_ID, &I2CBus::masterTxCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_I2C_RegisterCallback(handle_, HAL_I2C_MASTER_RX_COMPLETE_CB_ID, &I2CBus::masterRxCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_I2C_RegisterCallback(handle_, HAL_I2C_MEM_TX_COMPLETE_CB_ID, &I2CBus::memTxCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_I2C_RegisterCallback(handle_, HAL_I2C_MEM_RX_COMPLETE_CB_ID, &I2CBus::memRxCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_I2C_RegisterCallback(handle_, HAL_I2C_ERROR_CB_ID, &I2CBus::errorCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_I2C_RegisterCallback(handle_, HAL_I2C_ABORT_CB_ID, &I2CBus::abortCpltCallback) != HAL_OK) {
        registered = false;
    }

    if (!registered) {
        unregisterBus(this);
        return false;
    }

    registerBus(this);
    return true;
}

bool I2CBus::transferIt(I2CTransfer& transfer)
{
    return enqueueTransfer(transfer);
}

bool I2CBus::abort()
{
    if (!valid()) {
        return false;
    }

    const HAL_StatusTypeDef status = HAL_I2C_Master_Abort_IT(handle_, active_transfer_ != nullptr ? active_transfer_->device_address : 0U);
    I2CTransfer* transfer = clearActiveTransfer(false);
    if (queue_ != nullptr) {
        xQueueReset(queue_);
    }

    if (transfer != nullptr) {
        transfer->status = status;
        transfer->error_code = HAL_I2C_GetError(handle_);
        if (transfer->transmited_cb) {
            transfer->transmited_cb(transfer);
        }
    }

    return status == HAL_OK;
}

void I2CBus::masterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    I2CBus* bus = find(hi2c);
    if (bus != nullptr) {
        bus->completeTransfer(HAL_OK, true, true);
    }
}

void I2CBus::masterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    I2CBus* bus = find(hi2c);
    if (bus != nullptr) {
        bus->completeTransfer(HAL_OK, true, true);
    }
}

void I2CBus::memTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    I2CBus* bus = find(hi2c);
    if (bus != nullptr) {
        bus->completeTransfer(HAL_OK, true, true);
    }
}

void I2CBus::memRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    I2CBus* bus = find(hi2c);
    if (bus != nullptr) {
        bus->completeTransfer(HAL_OK, true, true);
    }
}

void I2CBus::errorCallback(I2C_HandleTypeDef* hi2c)
{
    I2CBus* bus = find(hi2c);
    if (bus != nullptr) {
        bus->completeTransfer(HAL_ERROR, true, true);
    }
}

void I2CBus::abortCpltCallback(I2C_HandleTypeDef* hi2c)
{
    I2CBus* bus = find(hi2c);
    if (bus != nullptr) {
        bus->completeTransfer(HAL_ERROR, true, true);
    }
}

} // namespace bsp
