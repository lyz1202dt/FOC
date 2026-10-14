#pragma once

#include "flash.hpp"

class Controller {
public:
    Controller(Flash &flash) : flash(flash) {}
    virtual ~Controller() = default;
    virtual void
        set_measure(const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool &rad_is_trustable) {};
    virtual bool get_output(float& v1, float& v2, float& v3) {
        v1 = 0.0f;
        v2 = 0.0f;
        v3 = 0.0f;
        return true;
    };
    virtual void controller_reference(void* ref,size_t size){};

    protected:
    Flash& flash;
};
