#pragma once

#include "controller.hpp"

//VOC————Voltage Open Circle 电压开环测试

class VOC : public Controller {
public:
    VOC(Flash& flash);
    ~VOC() = default;
    // 仅限任务上下文，在请求切入 TEST 前读取配置。
    bool load_config();
    void set_measure(
        const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable) override;
    void reset() override;
    bool get_output(float& v1, float& v2, float& v3) override;
    void controller_reference(void* ref, size_t size) override;

    int n{0};
    float offset{0.0f};

private:
    float vir_ecc_rad{0.0f};
    float measured_rad{0.0f};
    uint8_t use_virtue_rad{false};
    bool config_loaded_{false};
    bool measured_rad_valid_{false};
};
