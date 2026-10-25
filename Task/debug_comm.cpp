#include "debug_comm.hpp"

#include <cstddef>

extern "C" {
volatile float jscope_motor_position = 0.0F;
volatile float jscope_motor_velocity = 0.0F;
volatile float jscope_motor_current = 0.0F;
volatile uint32_t jscope_motor_status = 0U;
}

bool DebugComm::send_pack(Command* data) {
    if (data == nullptr) {
        return false;
    }
    if (data->cmd != 0x16U) {
        return true;
    }
    if (data->size < sizeof(MotorStatus)) {
        return false;
    }

    const auto* status = reinterpret_cast<const MotorStatus*>(data->data);
    jscope_motor_position = status->position;
    jscope_motor_velocity = status->velocity;
    jscope_motor_current = status->current;
    jscope_motor_status = status->status;
    return true;
}
