#pragma once

#include "controller.hpp"
#include "foc.hpp"
#include <cstdint>

// 调用方持有 FOC；MIT 与该 FOC 不可同时作为活动控制器。
class MIT : public Controller {
public:
    struct Reference {
        float position;  // 机械位置，rad，相对于 foc/offset，目标限幅至 ±mit/max_r 圈。
        float velocity;  // 机械角速度，rad/s。
        float current;   // 前馈 q 轴电流，A。
        float kp;        // A/rad。
        float kd;        // A/(rad/s)。
    };

    MIT(Flash& flash, FOC* foc);
    // 仅限任务上下文；读取配置后整体发布，不在控制中断中访问 Flash。
    void load_config();
    void reset() override;
    void set_measure(
        const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad,
        const bool& rad_is_trustable) override;
    bool get_output(float& v1, float& v2, float& v3) override;
    // 接收 Reference 或依次为 position/velocity/current/kp/kd 的五个 float。
    void controller_reference(void* ref, size_t size) override;
    // 与 FOC 相同：返回实际 q 轴电流（一个 float）。
    void controller_status(void* ref, size_t size) override;

private:
    FOC* foc_;
    Reference reference_{0.0f, 3.0f, 0.0f, 0.0f, 1.0f};
    float position_limit_{25.13274122871834590768f};  // 默认 ±4 圈，单位 rad。
    float position_{0.0f};
    float velocity_{0.0f};
    float previous_rad_{0.0f};
    uint64_t previous_us_{0U};
    bool has_sample_{false};
    bool sample_valid_{false};
};
