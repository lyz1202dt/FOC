#include "SPIBus.hpp"

#include "hardware.hpp"

namespace BSP {

SPIBus::SPIBus(SPI_HandleTypeDef* handle, GPIO_TypeDef* default_cs_port, uint16_t default_cs_pin)
    : handle_(handle), default_cs_port_(default_cs_port), default_cs_pin_(default_cs_pin) {}

bool SPIBus::transfer(const uint8_t* tx_data, uint8_t* rx_data, uint16_t length,
                      Callback callback, void* context,
                      GPIO_TypeDef* cs_port, uint16_t cs_pin) {
    Transaction transaction{};
    transaction.tx_data = tx_data;
    transaction.rx_data = rx_data;
    transaction.length = length;
    transaction.cs_port = cs_port ? cs_port : default_cs_port_;
    transaction.cs_pin = cs_port ? cs_pin : default_cs_pin_;
    transaction.callback = callback;
    transaction.context = context;
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
    bool accepted = false;

    __disable_irq();
    if (count_ < kTransactionCapacity) {
        queue_[tail_] = transaction;
        tail_ = (tail_ + 1U) % kTransactionCapacity;
        ++count_;
        accepted = true;
        should_start = !active_;
        if (should_start) {
            active_ = true;
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

void SPIBus::start_next() {
    if (active_transaction_.cs_port && active_transaction_.cs_pin != 0U) {
        HAL_GPIO_WritePin(active_transaction_.cs_port, active_transaction_.cs_pin, GPIO_PIN_RESET);
    }

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_IT(handle_,
                                                           active_transaction_.tx_data,
                                                           active_transaction_.rx_data,
                                                           active_transaction_.length);
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

    if (finished.cs_port && finished.cs_pin != 0U) {
        HAL_GPIO_WritePin(finished.cs_port, finished.cs_pin, GPIO_PIN_SET);
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
