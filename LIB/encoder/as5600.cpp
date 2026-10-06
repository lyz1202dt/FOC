#include "as5600.hpp"

#include "i2c.hpp"

namespace {

constexpr uint16_t kAs5600Address = static_cast<uint16_t>(0x36u << 1);
constexpr uint16_t kAs5600AngleRegister = 0x0Cu;

} // namespace

AS5600::AS5600(BSP::I2CBus& bus)
    : bus_(bus) {
}

bool AS5600::init() {
    if (started_) {
        return true;
    }

    started_ = sensor_update();
    return started_;
}

bool AS5600::sensor_update() {
    transfer_.operation = bsp::I2COperation::MemoryRead;
    transfer_.device_address = kAs5600Address;
    transfer_.memory_address = kAs5600AngleRegister;
    transfer_.memory_address_size = I2C_MEMADD_SIZE_8BIT;
    transfer_.rx_data = &recv_buffer[0];
    transfer_.tx_data = nullptr;
    transfer_.size = 2U;
    transfer_.param = this;
    transfer_.transmited_cb = [](bsp::I2CTransfer* transfer) {
        AS5600::on_read_complete(transfer->param, transfer->status == HAL_OK && transfer->error_code == HAL_I2C_ERROR_NONE);
    };
    return bus_.transferIt(transfer_);
}

void AS5600::on_read_complete(void* ctx, bool success) {
    AS5600* self = static_cast<AS5600*>(ctx);
    if (success) {
        float angle = static_cast<float>((((uint16_t)(self->recv_buffer[0] & 0x0Fu)) << 8) |
                                         static_cast<uint16_t>(self->recv_buffer[1]));
        self->record_value(angle * 3.14159265f * 2.0f / 4096.0f);
    }

    (void)self->sensor_update();
}
