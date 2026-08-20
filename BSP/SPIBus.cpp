#include "SPIBus.hpp"

#include "hardware.hpp"

namespace BSP {

SPIBus::SPIBus(SPI_HandleTypeDef* handle, Gpio* default_cs)
    : handle_(handle)
    , default_cs_(default_cs) {
}


bool SPIBus::init() {
    return true;
}


bool SPIBus::transfer(const uint8_t* tx_data, uint8_t* rx_data, uint16_t length, Callback callback, void* context, Gpio* cs) {
    Transaction transaction{};
    transaction.tx_data  = tx_data;
    transaction.rx_data  = rx_data;
    transaction.length   = length;
    transaction.cs       = cs ? cs : default_cs_;
    transaction.callback = callback;
    transaction.context  = context;
    return enqueue(transaction);
}

void SPIBus::on_transfer_complete(SPI_HandleTypeDef* handle) {
    if (!owns(handle)) {
        return;
    }

    complete_active(true);
}

void SPIBus::on_error(SPI_HandleTypeDef* handle) {
    if (!owns(handle)) {
        return;
    }

    complete_active(false);
}

bool SPIBus::enqueue(const Transaction& transaction) {
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

bool SPIBus::transfer_blocking(const uint8_t* tx_data, uint8_t* rx_data, uint16_t length, uint32_t timeout_ms, Gpio* cs) {
    if (!handle_ || !tx_data || !rx_data || length == 0U) {
        return false;
    }

    bool accepted = false;
    __disable_irq();
    if (!active_ && count_ == 0U) {
        active_  = true;
        accepted = true;
    }
    __enable_irq();

    if (!accepted) {
        return false;
    }

    Gpio* transaction_cs = cs ? cs : default_cs_;
    if (transaction_cs) {
        transaction_cs->write(false);
    }

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(handle_, const_cast<uint8_t*>(tx_data), rx_data, length, timeout_ms);

    if (transaction_cs) {
        transaction_cs->write(true);
    }

    bool should_start = false;
    __disable_irq();
    active_ = count_ > 0U;
    if (active_) {
        active_transaction_ = queue_[head_];
        should_start        = true;
    }
    __enable_irq();

    if (should_start) {
        start_next();
    }

    return status == HAL_OK;
}

void SPIBus::start_next() {
    if (active_transaction_.cs) {
        active_transaction_.cs->write(false);
    }

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive_IT(handle_, active_transaction_.tx_data, active_transaction_.rx_data, active_transaction_.length);
    if (status != HAL_OK) {
        complete_active(false);
    }
}

void SPIBus::complete_active(bool success) {
    Transaction finished;

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

    if (finished.cs) {
        finished.cs->write(true);
    }

    if (finished.callback) {
        finished.callback(finished.context, success);
    }

    __disable_irq();
    active_ = count_ > 0U;
    if (active_) {
        active_transaction_ = queue_[head_];
    }
    bool should_start = active_;
    __enable_irq();

    if (should_start) {
        start_next();
    }
}

bool SPIBus::owns(SPI_HandleTypeDef* handle) const {
    return handle_ == handle;
}

} // namespace BSP
