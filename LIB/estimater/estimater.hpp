#pragma once

#include "flash.hpp"

class Estimater {
public:
    Estimater(Flash &flash) : flash(flash) {}
    virtual ~Estimater() = default;
    virtual void set_raw_rad(float rad, bool available) {
        if (available) {
            measured_rad_ = rad;
        }
    }
    virtual void set_raw_vol(float vbus, float v1, float v2, float v3) {}
    virtual void set_raw_cur(float c1, float c2, float c3) {}
    virtual bool update(float& rad) {
        rad = measured_rad_;
        return is_available_;
    }
    virtual bool is_available() const {
        return is_available_;
    }
    uint32_t error_;
protected:
    Flash &flash;
    float measured_rad_{0.0f};
    bool is_available_{true};
};
