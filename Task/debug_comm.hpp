#pragma once

#include "comm.hpp"

// These symbols are intentionally plain C symbols so host-side probes such as
// J-Scope can find them without having to decode C++ names.
extern "C" {
extern volatile float jscope_motor_position;
extern volatile float jscope_motor_velocity;
extern volatile float jscope_motor_current;
extern volatile uint32_t jscope_motor_status;
}

class DebugComm final : public Comm {
public:
    bool init() override { return true; }
    bool send_pack(Command* data) override;
    int allow_send() override { return 1; }
};
