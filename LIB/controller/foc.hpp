#pragma once

#include "controller.hpp"

class FOC : public Controller {
public:
    FOC();
    virtual ~FOC()=default;
    void set_measure(const float& c1,
                     const float& c2,
                     const float& c3,
                     const float& vbus,
                     const float& rad,
                     const bool& rad_is_trustable) override;
    bool get_output(float& v1, float& v2, float& v3) override;
    void set_controller_reference(float position,float velocity,float torque,float kp,float kd)override;
private:
        float exp_iq_;
        float exp_id_;
};
