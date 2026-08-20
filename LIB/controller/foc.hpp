#pragma once

#include "controller.hpp"

class FOC : public Controller {
public:
    FOC();
    void set_measure(const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad) override;
    bool get_output(float& v1, float& v2, float& v3) override;
};
