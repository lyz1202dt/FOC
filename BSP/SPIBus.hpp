#pragma once

#include <cstddef>
#include <cstdint>

#include "spi.h"

namespace BSP {

class SPIBus {
public:
    using Callback = void (*)(void* context, bool success);

    static constexpr std::size_t kTransactionCapacity = 8;

    explicit SPIBus(SPI_HandleTypeDef* handle,
                    GPIO_TypeDef* default_cs_port = nullptr,
                    uint16_t default_cs_pin = 0);

    SPIBus(const SPIBus&) = delete;
    SPIBus& operator=(const SPIBus&) = delete;

    bool transfer(const uint8_t* tx_data, uint8_t* rx_data, uint16_t length,
                  Callback callback = nullptr, void* context = nullptr,
                  GPIO_TypeDef* cs_port = nullptr, uint16_t cs_pin = 0);

    void on_transfer_complete(SPI_HandleTypeDef* handle);
    void on_error(SPI_HandleTypeDef* handle);

private:
    struct Transaction {
        const uint8_t* tx_data{nullptr};
        uint8_t* rx_data{nullptr};
        uint16_t length{0};
        GPIO_TypeDef* cs_port{nullptr};
        uint16_t cs_pin{0};
        Callback callback{nullptr};
        void* context{nullptr};
    };

    bool enqueue(const Transaction& transaction);
    void start_next();
    void complete_active(bool success);
    bool owns(SPI_HandleTypeDef* handle) const;

    SPI_HandleTypeDef* handle_;
    GPIO_TypeDef* default_cs_port_;
    uint16_t default_cs_pin_;
    Transaction queue_[kTransactionCapacity];
    std::size_t head_{0};
    std::size_t tail_{0};
    std::size_t count_{0};
    bool active_{false};
    Transaction active_transaction_{};
};

} // namespace BSP
