#include "debug_comm.hpp"

#include <cstddef>

extern "C" {
volatile float jscope_motor_position = 0.0F;
volatile float jscope_motor_velocity = 0.0F;
volatile float jscope_motor_current = 0.0F;
volatile uint32_t jscope_motor_status = 0U;
/**Debug*/
volatile float jscope_phase_current_a = 0.0F;
volatile float jscope_phase_current_b = 0.0F;
volatile float jscope_phase_current_c = 0.0F;
volatile float jscope_bus_voltage=0.0F;
/**Debug*/
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
