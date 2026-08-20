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

    explicit SPIBus(SPI_HandleTypeDef* handle, Gpio* default_cs = nullptr);

    SPIBus(const SPIBus&)            = delete;
    SPIBus& operator=(const SPIBus&) = delete;

    bool init();
    bool transfer(
        const uint8_t* tx_data, uint8_t* rx_data, uint16_t length, Callback callback = nullptr, void* context = nullptr,
        Gpio* cs = nullptr);
    bool transfer_blocking(
        const uint8_t* tx_data, uint8_t* rx_data, uint16_t length, uint32_t timeout_ms, Gpio* cs = nullptr);

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
    };

    bool enqueue(const Transaction& transaction);
    void start_next();
    void complete_active(bool success);
    bool owns(SPI_HandleTypeDef* handle) const;

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
