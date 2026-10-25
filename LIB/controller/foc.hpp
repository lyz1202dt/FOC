#pragma once

#include "controller.hpp"
#include "pid.hpp"

class FOC : public Controller {
public:
    explicit FOC(Flash &flash);
    virtual ~FOC() = default;
    // 仅限任务上下文，在请求切入 FOC 前读取最新极对数与机械零位。
    bool load_config();
    void set_measure(
        const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable) override;
    void reset() override;
    bool get_output(float& v1, float& v2, float& v3) override;
    void controller_reference(void* ref,size_t size) override;
    void controller_status(void* ref,size_t size) override;

    int n{1};
    float offset{0.0f};
private:
    float exp_iq_{0.0f};
    float cur_iq_{0.0f};
    
    PID iq_pid,id_pid;
    float c1,c2,c3,vbus,rad{0.0};
    bool first_run{true};
    bool config_loaded_{false};
};
