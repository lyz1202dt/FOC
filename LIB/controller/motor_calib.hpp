#pragma once

#include "controller.hpp"
#include <cstdint>

class MotorCalib : public Controller {
public:
    struct Result {
        float resistance;
        float inductance;
        float kp;
        float ki;
        bool success;
    };

    explicit MotorCalib(Flash& flash);
    void reset() override;
    void set_measure(const float& c1, const float& c2, const float& c3, const float& vbus,
                     const float& rad, const bool& rad_is_trustable) override;
    bool get_output(float& v1, float& v2, float& v3) override;
    bool get_result(Result& result) const;

private:
    enum class Stage : uint8_t { Resistance, Inductance, Complete };
    static constexpr uint64_t kResistanceTimeUs = 500000U;
    static constexpr uint64_t kResistanceSettleUs = 250000U;
    static constexpr uint64_t kInductanceTimeUs = 1000000U;
    static constexpr uint64_t kHalfPeriodUs = 500U;
    static constexpr float kBandwidth = 1000.0f;

    Stage stage_{Stage::Resistance};
    Result result_{};
    uint64_t stage_start_us_{0U};
    uint64_t last_sample_us_{0U};
    float current_{0.0f};
    float vbus_{0.0f};
    float last_current_{0.0f};
    float current_sum_{0.0f};
    float signed_current_change_{0.0f};
    float signed_voltage_time_{0.0f};
    uint32_t current_samples_{0U};
    int8_t applied_sign_{0};
    bool started_{false};
    bool sample_valid_{false};
};
