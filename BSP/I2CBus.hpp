#pragma once

#include <cstddef>
#include <cstdint>

#include "i2c.h"

namespace BSP {

class I2CBus {
public:
    enum class MemoryAddressSize : uint16_t {
        Bits8 = I2C_MEMADD_SIZE_8BIT,
        Bits16 = I2C_MEMADD_SIZE_16BIT,
    };

    using Callback = void (*)(void* context, bool success);

    static constexpr std::size_t kTransactionCapacity = 8;

    explicit I2CBus(I2C_HandleTypeDef* handle);

    I2CBus(const I2CBus&) = delete;
    I2CBus& operator=(const I2CBus&) = delete;

    bool master_transmit(uint16_t device_address, const uint8_t* data, uint16_t length,
                         Callback callback = nullptr, void* context = nullptr);
    bool master_receive(uint16_t device_address, uint8_t* data, uint16_t length,
                        Callback callback = nullptr, void* context = nullptr);
    bool memory_write(uint16_t device_address, uint16_t memory_address, MemoryAddressSize memory_address_size,
                      const uint8_t* data, uint16_t length, Callback callback = nullptr, void* context = nullptr);
    bool memory_read(uint16_t device_address, uint16_t memory_address, MemoryAddressSize memory_address_size,
                     uint8_t* data, uint16_t length, Callback callback = nullptr, void* context = nullptr);

    void on_transfer_complete(I2C_HandleTypeDef* handle);
    void on_error(I2C_HandleTypeDef* handle);

private:
    enum class Operation : uint8_t {
        MasterTransmit,
        MasterReceive,
        MemoryWrite,
        MemoryRead,
    };

    struct Transaction {
        Operation operation{Operation::MasterTransmit};
        uint16_t device_address{0};
        uint16_t memory_address{0};
        MemoryAddressSize memory_address_size{MemoryAddressSize::Bits8};
        const uint8_t* tx_data{nullptr};
        uint8_t* rx_data{nullptr};
        uint16_t length{0};
        Callback callback{nullptr};
        void* context{nullptr};
    };

    bool enqueue(const Transaction& transaction);
    void start_next();
    void complete_active(bool success);
    bool owns(I2C_HandleTypeDef* handle) const;
    uint16_t to_hal_mem_address_size(MemoryAddressSize memory_address_size) const;

    I2C_HandleTypeDef* handle_;
    Transaction queue_[kTransactionCapacity];
    std::size_t head_{0};
    std::size_t tail_{0};
    std::size_t count_{0};
    bool active_{false};
    Transaction active_transaction_{};
};

} // namespace BSP
