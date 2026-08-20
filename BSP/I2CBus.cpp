#include "I2CBus.hpp"

#include "hardware.hpp"

namespace BSP {

I2CBus::I2CBus(I2C_HandleTypeDef* handle)
    : handle_(handle) {
}

bool I2CBus::init() {
    return true;
}

bool I2CBus::master_transmit(uint16_t device_address, const uint8_t* data, uint16_t length, Callback callback, void* context) {
    Transaction transaction{};
    transaction.operation      = Operation::MasterTransmit;
    transaction.device_address = device_address;
    transaction.tx_data        = data;
    transaction.length         = length;
    transaction.callback       = callback;
    transaction.context        = context;
    return enqueue(transaction);
}

bool I2CBus::master_receive(uint16_t device_address, uint8_t* data, uint16_t length, Callback callback, void* context) {
    Transaction transaction{};
    transaction.operation      = Operation::MasterReceive;
    transaction.device_address = device_address;
    transaction.rx_data        = data;
    transaction.length         = length;
    transaction.callback       = callback;
    transaction.context        = context;
    return enqueue(transaction);
}

bool I2CBus::memory_write(
    uint16_t device_address, uint16_t memory_address, MemoryAddressSize memory_address_size, const uint8_t* data, uint16_t length,
    Callback callback, void* context) {
    Transaction transaction{};
    transaction.operation           = Operation::MemoryWrite;
    transaction.device_address      = device_address;
    transaction.memory_address      = memory_address;
    transaction.memory_address_size = memory_address_size;
    transaction.tx_data             = data;
    transaction.length              = length;
    transaction.callback            = callback;
    transaction.context             = context;
    return enqueue(transaction);
}

bool I2CBus::memory_read(
    uint16_t device_address, uint16_t memory_address, MemoryAddressSize memory_address_size, uint8_t* data, uint16_t length,
    Callback callback, void* context) {
    Transaction transaction{};
    transaction.operation           = Operation::MemoryRead;
    transaction.device_address      = device_address;
    transaction.memory_address      = memory_address;
    transaction.memory_address_size = memory_address_size;
    transaction.rx_data             = data;
    transaction.length              = length;
    transaction.callback            = callback;
    transaction.context             = context;
    return enqueue(transaction);
}

void I2CBus::on_transfer_complete(I2C_HandleTypeDef* handle) {
    if (!owns(handle)) {
        return;
    }

    complete_active(true);
}

void I2CBus::on_error(I2C_HandleTypeDef* handle) {
    if (!owns(handle)) {
        return;
    }

    complete_active(false);
}

bool I2CBus::enqueue(const Transaction& transaction) {
    bool should_start = false;
    bool accepted     = false;

    __disable_irq();
    if (count_ < kTransactionCapacity) {
        queue_[tail_] = transaction;
        tail_         = (tail_ + 1U) % kTransactionCapacity;
        ++count_;
        accepted     = true;
        should_start = !active_;
        if (should_start) {
            active_             = true;
            active_transaction_ = queue_[head_];
        }
    }
    __enable_irq();

    if (!accepted) {
        return false;
    }

    if (should_start) {
        start_next();
    }

    return true;
}

void I2CBus::start_next() {
    HAL_StatusTypeDef status = HAL_ERROR;

    switch (active_transaction_.operation) {
    case Operation::MasterTransmit:
        status = HAL_I2C_Master_Transmit_IT(
            handle_, active_transaction_.device_address, const_cast<uint8_t*>(active_transaction_.tx_data), active_transaction_.length);
        break;
    case Operation::MasterReceive:
        status =
            HAL_I2C_Master_Receive_IT(handle_, active_transaction_.device_address, active_transaction_.rx_data, active_transaction_.length);
        break;
    case Operation::MemoryWrite:
        status = HAL_I2C_Mem_Write_IT(
            handle_, active_transaction_.device_address, active_transaction_.memory_address,
            to_hal_mem_address_size(active_transaction_.memory_address_size), const_cast<uint8_t*>(active_transaction_.tx_data),
            active_transaction_.length);
        break;
    case Operation::MemoryRead:
        status = HAL_I2C_Mem_Read_IT(
            handle_, active_transaction_.device_address, active_transaction_.memory_address,
            to_hal_mem_address_size(active_transaction_.memory_address_size), active_transaction_.rx_data, active_transaction_.length);
        break;
    }

    if (status != HAL_OK) {
        complete_active(false);
    }
}

void I2CBus::complete_active(bool success) {
    Transaction finished;
    bool should_start = false;

    __disable_irq();
    if (!active_) {
        __enable_irq();
        return;
    }

    finished = active_transaction_;
    if (count_ > 0U) {
        head_ = (head_ + 1U) % kTransactionCapacity;
        --count_;
    }
    __enable_irq();

    if (finished.callback) {
        finished.callback(finished.context, success);
    }

    __disable_irq();
    active_ = count_ > 0U;
    if (active_) {
        active_transaction_ = queue_[head_];
    }
    should_start = active_;
    __enable_irq();

    if (should_start) {
        start_next();
    }
}

bool I2CBus::owns(I2C_HandleTypeDef* handle) const {
    return handle_ == handle;
}

uint16_t I2CBus::to_hal_mem_address_size(MemoryAddressSize memory_address_size) const {
    return static_cast<uint16_t>(memory_address_size);
}

} // namespace BSP
