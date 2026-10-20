#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "driver.hpp"
#include "gpio.hpp"
#include "measure.hpp"
#include "spi.hpp"

namespace BSP {
class SVPWM;
}

class DRV8301 : public MotorDriver {
public:
    enum FaultType_e : uint32_t {
        FaultType_NoFault = 0U,

        FaultType_FETLC_OC = (1UL << 0),
        FaultType_FETHC_OC = (1UL << 1),
        FaultType_FETLB_OC = (1UL << 2),
        FaultType_FETHB_OC = (1UL << 3),
        FaultType_FETLA_OC = (1UL << 4),
        FaultType_FETHA_OC = (1UL << 5),
        FaultType_OTW = (1UL << 6),
        FaultType_OTSD = (1UL << 7),
        FaultType_PVDD_UV = (1UL << 8),
        FaultType_GVDD_UV = (1UL << 9),
        FaultType_FAULT = (1UL << 10),
        FaultType_GVDD_OV = (1UL << 23),

        FaultType_CurrentSenseSaturation = (1UL << 30),
        FaultType_SPI = (1UL << 31),
    };

    struct StatusRegisters {
        uint16_t status1{0};
        uint16_t status2{0};
    };

    DRV8301(Flash& flash, BSP::SPIBus& spi, BSP::SVPWM& svpwm, BSP::Measure& phase_current_measure,
            BSP::Gpio& enable_gate, BSP::Gpio& fault, BSP::Gpio& dc_cal);

    bool config(float requested_gain, float* actual_gain = nullptr);
    // 配置芯片和 PWM 外设，保持功率输出关闭；不启动测量。
    bool init() override;
    bool enable_pwm(bool enable) override;
    bool enable_measure(bool enable) override;
    bool is_ready() override;
    int lookup_reason(std::string& reason) override;

    bool get_vbus_voltage(float& vol) override;
    bool get_phrase_current(float& c1, float& c2, float& c3) override;
    bool set_phrase_voltage(float v1, float v2, float v3) override;

    bool self_check() override;

    FaultType_e get_error();
    bool read_status(StatusRegisters& status);
    bool clear_faults();

private:
    enum CtrlMode_e : uint16_t {
        DRV8301_CtrlMode_Read = (1U << 15),
        DRV8301_CtrlMode_Write = (0U << 15),
    };

    enum RegName_e : uint16_t {
        kRegNameStatus1 = (0U << 11),
        kRegNameStatus2 = (1U << 11),
        kRegNameControl1 = (2U << 11),
        kRegNameControl2 = (3U << 11),
    };

    enum class State : uint8_t {
        Uninitialized,
        StartupChecks,
        Ready,
    };

    struct RegisterFile {
        uint16_t control_register_1{0};
        uint16_t control_register_2{0};
    };

    static uint16_t build_ctrl_word(CtrlMode_e ctrl_mode, RegName_e reg_name, uint16_t data);

    bool read_reg(RegName_e reg_name, uint16_t& data);
    bool write_reg(RegName_e reg_name, uint16_t data);
    bool transfer_word(uint16_t tx_word, uint16_t* rx_word);
    float phase_current_from_adc(uint16_t raw, uint16_t offset) const;

    BSP::SPIBus& spi_;
    BSP::SVPWM& svpwm_;
    BSP::Measure& measure_;
    BSP::Gpio& enable_gate_;
    BSP::Gpio& fault_;
    BSP::Gpio& dc_cal_;
    RegisterFile regs_{};
    State state_{State::Uninitialized};
    float current_gain_{20.0f};
    uint16_t current_offsets_[2]{2048U, 2048U};
    uint32_t last_fault_{FaultType_NoFault};
    uint8_t tx_buf_[2]{};
    uint8_t rx_buf_[2]{};
    bool measure_started_{false};
    std::atomic<bool> current_calibrated_{false};
};
