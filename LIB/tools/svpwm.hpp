#pragma once

#include "../tools/flash.hpp"

class SVPWM {
public:
    SVPWM();
    void load_config();
    void set_voltage(float dc_bus, float v1,float v2,float v3);

private:
    Flash flash;
};