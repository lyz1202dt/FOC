#include "as5600.hpp"

#include "I2CBus.hpp"

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
    return bus_.memory_read(
        kAs5600Address,
        kAs5600AngleRegister,
        BSP::I2CBus::MemoryAddressSize::Bits8,
        &recv_buffer[0],
        2,
        &AS5600::on_read_complete,
        this);
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
