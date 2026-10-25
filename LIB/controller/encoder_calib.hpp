#pragma once

#include "controller.hpp"
#include <cstdint>

class EncoderCalib : public Controller {
public:
    explicit EncoderCalib(Flash& flash);
    void reset() override;
    void set_measure(
        const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable) override;
    bool get_output(float& v1, float& v2, float& v3) override;
    // 任务上下文读取 ISR 锁存的机械零位；不访问 Flash。
    bool get_offset(float& offset) const;

private:
    static constexpr uint64_t kAlignmentTimeUs = 2000000U;
    float measured_rad_{0.0f};
    float offset_{0.0f};
    uint64_t alignment_start_us_{0U};
    bool sample_valid_{false};
    bool aligning_{false};
    bool complete_{false};
};
