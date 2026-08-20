#include "SPIBus.hpp"

#include "hardware.hpp"

namespace BSP {

SPIBus::SPIBus(SPI_HandleTypeDef* handle, Gpio* default_cs)
    : handle_(handle)
    , default_cs_(default_cs) {
}


bool SPIBus::init() {
    return handle_ && handle_->Instance;
}


bool SPIBus::transfer(const uint8_t* tx_data, uint8_t* rx_data, uint16_t length, Callback callback, void* context,
                      const TransferOptions& options, Gpio* cs) {
    if (!handle_ || !tx_data || !rx_data || length == 0U) {
        return false;
    }

    if (!is_valid_options(options)) {
        return false;
    }

    Transaction transaction{};
    transaction.tx_data  = tx_data;
    transaction.rx_data  = rx_data;
    transaction.length   = length;
    transaction.cs       = cs ? cs : default_cs_;
    transaction.callback = callback;
    transaction.context  = context;
    transaction.options  = options;
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

bool SPIBus::transfer_blocking(const uint8_t* tx_data, uint8_t* rx_data, uint16_t length, uint32_t timeout_ms,
                               const TransferOptions& options, Gpio* cs) {
    if (!handle_ || !tx_data || !rx_data || length == 0U) {
        return false;
    }

    if (!is_valid_options(options)) {
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

    if (!configure_for_transfer(options)) {
        finish_blocking_transaction();
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

    finish_blocking_transaction();

    return status == HAL_OK;
}

void SPIBus::start_next() {
    if (!configure_for_transfer(active_transaction_.options)) {
        complete_active(false);
        return;
    }

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

void SPIBus::finish_blocking_transaction() {
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
}

bool SPIBus::owns(SPI_HandleTypeDef* handle) const {
    return handle_ == handle;
}

bool SPIBus::configure_for_transfer(const TransferOptions& options) {
    if (!handle_ || !handle_->Instance || !is_valid_options(options)) {
        return false;
    }

    if (handle_->State != HAL_SPI_STATE_READY || READ_BIT(handle_->Instance->SR, SPI_SR_BSY) != 0U) {
        return false;
    }

    __HAL_SPI_DISABLE(handle_);
    MODIFY_REG(
        handle_->Instance->CR1,
        SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_LSBFIRST,
        (options.clk_polarity & SPI_CR1_CPOL) | (options.clk_phase & SPI_CR1_CPHA) |
            (options.first_bit & SPI_CR1_LSBFIRST));
    MODIFY_REG(
        handle_->Instance->CR2,
        SPI_CR2_DS | SPI_CR2_FRXTH,
        (options.data_size & SPI_CR2_DS) | rx_fifo_threshold_for(options.data_size));

    handle_->Init.DataSize    = options.data_size;
    handle_->Init.CLKPolarity = options.clk_polarity;
    handle_->Init.CLKPhase    = options.clk_phase;
    handle_->Init.FirstBit    = options.first_bit;
    return true;
}

bool SPIBus::is_valid_options(const TransferOptions& options) {
    if (!IS_SPI_DATASIZE(options.data_size)) {
        return false;
    }
    if (!IS_SPI_CPOL(options.clk_polarity)) {
        return false;
    }
    if (!IS_SPI_CPHA(options.clk_phase)) {
        return false;
    }
    if (!IS_SPI_FIRST_BIT(options.first_bit)) {
        return false;
    }
    return true;
}

uint32_t SPIBus::rx_fifo_threshold_for(uint32_t data_size) {
    return data_size > SPI_DATASIZE_8BIT ? SPI_RXFIFO_THRESHOLD_HF : SPI_RXFIFO_THRESHOLD_QF;
}

} // namespace BSP
