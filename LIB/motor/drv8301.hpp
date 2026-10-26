#pragma once

#include <atomic>
#include <cstdint>

#include "adc.hpp"
#include "driver.hpp"
#include "gpio.hpp"
#include "samplegroup.hpp"
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

    DRV8301(Flash& flash, bsp::SpiBus& spi, BSP::Gpio& cs, BSP::SVPWM& svpwm, bsp::AdcChannel& adc,
            BSP::Gpio& enable_gate, BSP::Gpio& fault, BSP::Gpio& dc_cal);

    bool config(float requested_gain, float* actual_gain = nullptr);
    // 配置芯片和 PWM 外设并启动 ADC 注入组，保持功率输出关闭；TIM1 由调用方启动。
    bool init() override;
    bool enable_pwm(bool enable) override;
    bool is_ready() override;

    bool get_vbus_voltage(float& vol) override;
    bool get_phrase_current(float& c1, float& c2, float& c3) override;
    bool set_phrase_voltage(float v1, float v2, float v3) override;

    // 任务上下文零电流校准：转子须静止，调用方保持 TIM1 采样时基运行。
    // 校准使用正常输入（DC_CAL 关闭），执行后功率输出保持关闭。
    bool self_check() override;

    // 寄存器操作仅限任务上下文；并发请求或在途传输未完成时返回失败。
    FaultType_e get_error();
    void update_error() override;
    bool read_status(StatusRegisters& status);
    bool clear_faults();

private:
    enum AdcSampleIndex {
        kPhaseB,
        kPhaseC,
        kVbus,
        kAdcSampleCount,
    };

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
        Fault,
    };

    struct RegisterFile {
        uint16_t control_register_1{0};
        uint16_t control_register_2{0};
    };

    static uint16_t build_ctrl_word(CtrlMode_e ctrl_mode, RegName_e reg_name, uint16_t data);

    bool start_adc_sampling();
    bool read_reg(RegName_e reg_name, uint16_t& data);
    bool write_reg(RegName_e reg_name, uint16_t data);
    bool transfer_register(uint16_t command, uint16_t* response);
    float phase_current_from_adc(uint16_t raw, uint16_t offset) const;

    bsp::SpiBus& spi_;
    BSP::Gpio& cs_;
    BSP::SVPWM& svpwm_;
    bsp::AdcChannel& adc_;
    SampleGroup<uint16_t, kAdcSampleCount> adc_samples_;
    float current_gain_{20.0f};
    uint16_t current_offsets_[2]{2048U, 2048U};

    BSP::Gpio& enable_gate_;
    BSP::Gpio& fault_;
    BSP::Gpio& dc_cal_;
    RegisterFile regs_{};
    std::atomic<State> state_{State::Uninitialized};
    bsp::SpiTransferEvent transfer_{};
    uint8_t tx_buf_[2]{};
    uint8_t rx_buf_[2]{};
    // register_busy_ 串行化整个寄存器事务；transfer_pending_ 保护异步缓冲区至回调结束。
    std::atomic<bool> register_busy_{false};
    std::atomic<bool> transfer_pending_{false};
    bsp::SpiTransferStatus transfer_status_{bsp::SpiTransferStatus::Ok};
    bool adc_started_{false};
};
