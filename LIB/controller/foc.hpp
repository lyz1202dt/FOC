#pragma once

#include "controller.hpp"
#include "pid.hpp"

class FOC : public Controller {
public:
    FOC();
    virtual ~FOC() = default;
    void set_measure(
        const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable) override;
    bool get_output(float& v1, float& v2, float& v3) override;
    void set_controller_reference(void* ref) override;

    int n;
    float offset;
private:
    float exp_iq_{0.0f};
    
    PID iq_pid,id_pid;
    float c1,c2,c3,vbus,rad{0.0};
    bool first_run{true};
};
