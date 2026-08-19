#pragma once

#include "../encoder/encoder.hpp"
#include "../tools/drv8301.hpp"
#include "../tools/measure.hpp"
#include "../tools/svpwm.hpp"
#include "../tools/flash.hpp"
#include "comm.hpp"

class Motor{
public:
    Motor();
    bool update();  //在中断中调用

private:
    Encoder* encoder;
    DRV8301 drv8301;
    Flash &flash;
    PhaseCurrentMeasure current_measure;
    SVPWM svpwm;
};