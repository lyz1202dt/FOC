#pragma once

#include <cstddef>
#include <cstdint>

#include "gpio.hpp"
#include "spi.h"

namespace BSP {

class SPIBus {
public:
    using Callback = void (*)(void* context, bool success);

    static constexpr std::size_t kTransactionCapacity = 8;

    struct TransferOptions {
        constexpr TransferOptions(uint32_t data_size_, uint32_t clk_polarity_, uint32_t clk_phase_, uint32_t first_bit_)
            : data_size(data_size_)
            , clk_polarity(clk_polarity_)
            , clk_phase(clk_phase_)
            , first_bit(first_bit_) {
        }

        uint32_t data_size;
        uint32_t clk_polarity;
        uint32_t clk_phase;
        uint32_t first_bit;
    };

    explicit SPIBus(SPI_HandleTypeDef* handle, Gpio* default_cs = nullptr);

    SPIBus(const SPIBus&)            = delete;
    SPIBus& operator=(const SPIBus&) = delete;

    bool init();
    bool transfer(
        const uint8_t* tx_data, uint8_t* rx_data, uint16_t length, Callback callback, void* context,
        const TransferOptions& options, Gpio* cs = nullptr);
    bool transfer_blocking(
        const uint8_t* tx_data, uint8_t* rx_data, uint16_t length, uint32_t timeout_ms,
        const TransferOptions& options, Gpio* cs = nullptr);

    void on_transfer_complete(SPI_HandleTypeDef* handle);
    void on_error(SPI_HandleTypeDef* handle);

private:
    struct Transaction {
        const uint8_t* tx_data{nullptr};
        uint8_t* rx_data{nullptr};
        uint16_t length{0};
        Gpio* cs{nullptr};
        Callback callback{nullptr};
        void* context{nullptr};
        TransferOptions options{0U, 0U, 0U, 0U};
    };

    bool enqueue(const Transaction& transaction);
    void start_next();
    void complete_active(bool success);
    void finish_blocking_transaction();
    bool owns(SPI_HandleTypeDef* handle) const;
    bool configure_for_transfer(const TransferOptions& options);
    static bool is_valid_options(const TransferOptions& options);
    static uint32_t rx_fifo_threshold_for(uint32_t data_size);

    SPI_HandleTypeDef* handle_;
    Gpio* default_cs_;
    Transaction queue_[kTransactionCapacity];
    std::size_t head_{0};
    std::size_t tail_{0};
    std::size_t count_{0};
    bool active_{false};
    Transaction active_transaction_{};
};

} // namespace BSP
