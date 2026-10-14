#include "drv8301.hpp"

#include "spi.hpp"
#include "gpio.hpp"
#include "hardware.hpp"
#include "measure.hpp"
#include "svpwm.hpp"

#include "stm32g4xx_hal.h"

namespace {

constexpr uint32_t kSampleFreshnessUs       = 10000U;
constexpr uint32_t kSpiTimeoutMs            = 10U;
constexpr float kAdcReferenceVolts          = 3.3f;
constexpr float kAdcFullScale               = 4096.0f;
constexpr float kCurrentSenseMinVolt        = 0.3f;
constexpr float kCurrentSenseMaxVolt        = 3.0f;
constexpr float kCurrentSenseMaxSwingVolts  = 1.35f;
constexpr float kDefaultCurrentGain         = 20.0f;
constexpr float kShuntResistanceOhms        = 0.001f;
constexpr float kVbusDividerRatio           = 11.0f; // R5=20K, R9=2K on SCH_Schematic1_2026-08-16.pdf.

constexpr uint16_t kCurrentAdcLowerBound =
    static_cast<uint16_t>(kAdcFullScale * kCurrentSenseMinVolt / kAdcReferenceVolts);
constexpr uint16_t kCurrentAdcUpperBound =
    static_cast<uint16_t>(kAdcFullScale * kCurrentSenseMaxVolt / kAdcReferenceVolts);

void append_reason(std::string& reason, const char* text) {
    if (!reason.empty()) {
        reason += ", ";
    }
    reason += text;
}

void delay_us(uint32_t us) {
    if (us == 0U) {
        return;
    }

    const uint64_t start = BSP::get_time_us();
    while ((BSP::get_time_us() - start) < us) {
        __NOP();
    }
}

} // namespace

DRV8301::DRV8301(Flash& flash, BSP::SPIBus& spi, BSP::SVPWM& svpwm, BSP::Measure& phase_current_measure,
                 BSP::Gpio& enable_gate, BSP::Gpio& fault, BSP::Gpio& dc_cal)
    : MotorDriver(flash)
    , spi_(spi)
    , svpwm_(svpwm)
    , measure_(phase_current_measure)
    , enable_gate_(enable_gate)
    , fault_(fault)
    , dc_cal_(dc_cal) {
    config(kDefaultCurrentGain);
}

bool DRV8301::config(float requested_gain, float* actual_gain) {
    static constexpr float kGainChoices[] = {10.0f, 20.0f, 40.0f, 80.0f};

    uint16_t gain_setting = 3U;
    while (gain_setting > 0U && kGainChoices[gain_setting] > requested_gain) {
        --gain_setting;
    }

    RegisterFile new_regs{};
    new_regs.control_register_1 =
        (21U << 6)  // OC_ADJ_SET: ODrive default, high over-current threshold.
        | (1U << 4) // OCP_MODE: latch shutdown.
        | (0U << 3) // PWM_MODE: 6 PWM inputs.
        | (0U << 2) // GATE_RESET: normal operation.
        | (0U << 0); // GATE_CURRENT: 1.7A peak.

    new_regs.control_register_2 =
        (0U << 6)               // OC_TOFF: cycle-by-cycle.
        | (0U << 4)             // DC_CAL_CH1/2: normal operation.
        | (gain_setting << 2)   // GAIN: 10/20/40/80 V/V.
        | (0U << 0);            // OCTW_MODE: over-current and over-temperature.

    const bool regs_changed =
        regs_.control_register_1 != new_regs.control_register_1 ||
        regs_.control_register_2 != new_regs.control_register_2;

    regs_ = new_regs;
    current_gain_ = kGainChoices[gain_setting];
    if (actual_gain) {
        *actual_gain = current_gain_;
    }

    if (regs_changed && state_ == State::Ready) {
        state_ = State::Uninitialized;
        enabled_ = false;
        svpwm_.set_voltage(24.0f, 0.0f,0.0f,0.0f);
        enable_gate_.write(false);
    }

    return true;
}

bool DRV8301::init() {
    if (state_ == State::Ready) {
        return true;
    }

    state_ = State::Uninitialized;
    enabled_ = false;
    last_fault_ = FaultType_NoFault;
    svpwm_.set_voltage(24.0f, 0.0f,0.0f,0.0f);
    dc_cal_.write(false);
    enable_gate_.write(false);

    delay_us(40); // DRV8301 reset low time is only tens of us.
    enable_gate_.write(true);
    HAL_Delay(20); // t_spi_ready max is 10ms.

    // ODrive writes control register 1 multiple times because the first write
    // can be ignored on some boards.
    bool wrote_regs =
        write_reg(kRegNameControl1, regs_.control_register_1) &&
        write_reg(kRegNameControl1, regs_.control_register_1) &&
        write_reg(kRegNameControl1, regs_.control_register_1) &&
        write_reg(kRegNameControl1, regs_.control_register_1) &&
        write_reg(kRegNameControl1, regs_.control_register_1) &&
        write_reg(kRegNameControl2, regs_.control_register_2);

    if (!wrote_regs) {
        enable_gate_.write(false);
        last_fault_ = FaultType_SPI;
        return false;
    }

    delay_us(100);
    state_ = State::StartupChecks;

    uint16_t value = 0U;
    const bool regs_verified =
        read_reg(kRegNameControl1, value) && value == regs_.control_register_1 &&
        read_reg(kRegNameControl2, value) && value == regs_.control_register_2;
    if (!regs_verified) {
        state_ = State::Uninitialized;
        enable_gate_.write(false);
        last_fault_ = FaultType_SPI;
        return false;
    }

    FaultType_e fault = get_error();
    if (fault != FaultType_NoFault) {
        state_ = State::Uninitialized;
        enabled_ = false;
        svpwm_.set_voltage(24.0f, 0.0f,0.0f,0.0f);
        last_fault_ = fault;
        return false;
    }

    if (!fault_.read()) {
        state_ = State::Uninitialized;
        enabled_ = false;
        svpwm_.set_voltage(24.0f, 0.0f,0.0f,0.0f);
        last_fault_ = FaultType_FAULT;
        return false;
    }

    if (!calibrate_current_offsets()) {
        state_ = State::Uninitialized;
        enabled_ = false;
        svpwm_.set_voltage(24.0f, 0.0f,0.0f,0.0f);
        last_fault_ = FaultType_CurrentSenseSaturation;
        return false;
    }

    state_ = State::Ready;
    enabled_ = true;
    last_fault_ = FaultType_NoFault;
    return true;
}

bool DRV8301::is_ready() {
    do_checks();
    return state_ == State::Ready && enabled_;
}

bool DRV8301::set_enable(bool enable) {
    if (!enable) {
        enabled_ = false;
        state_ = State::Uninitialized;
        svpwm_.set_voltage(24.0f, 0.0f,0.0f,0.0f);
        enable_gate_.write(false);
        return true;
    }

    if (state_ != State::Ready) {
        return init();
    }

    enabled_ = true;
    enable_gate_.write(true);
    return true;
}

int DRV8301::lookup_reason(std::string& reason) {
    reason.clear();

    FaultType_e fault = static_cast<FaultType_e>(last_fault_);
    if (fault == FaultType_NoFault && state_ == State::Ready && enabled_) {
        fault = get_error();
    }
    if (fault == FaultType_NoFault && state_ == State::Ready && enabled_ && !fault_.read()) {
        fault = FaultType_FAULT;
    }

    if (fault == FaultType_NoFault) {
        if (!enabled_) {
            reason = "DRV8301 disabled";
            return 1;
        }

        if (state_ != State::Ready) {
            reason = "DRV8301 not initialized";
            return -1;
        }

        reason = "OK";
        return 0;
    }

    if (fault & FaultType_SPI) {
        append_reason(reason, "SPI communication failed");
    }
    if (fault & FaultType_FETLC_OC) {
        append_reason(reason, "phase C low-side over-current");
    }
    if (fault & FaultType_FETHC_OC) {
        append_reason(reason, "phase C high-side over-current");
    }
    if (fault & FaultType_FETLB_OC) {
        append_reason(reason, "phase B low-side over-current");
    }
    if (fault & FaultType_FETHB_OC) {
        append_reason(reason, "phase B high-side over-current");
    }
    if (fault & FaultType_FETLA_OC) {
        append_reason(reason, "phase A low-side over-current");
    }
    if (fault & FaultType_FETHA_OC) {
        append_reason(reason, "phase A high-side over-current");
    }
    if (fault & FaultType_OTW) {
        append_reason(reason, "over-temperature warning");
    }
    if (fault & FaultType_OTSD) {
        append_reason(reason, "over-temperature shutdown");
    }
    if (fault & FaultType_PVDD_UV) {
        append_reason(reason, "PVDD under-voltage");
    }
    if (fault & FaultType_GVDD_UV) {
        append_reason(reason, "GVDD under-voltage");
    }
    if (fault & FaultType_GVDD_OV) {
        append_reason(reason, "GVDD over-voltage");
    }
    if (fault & FaultType_CurrentSenseSaturation) {
        append_reason(reason, "current sense saturation");
    }
    if (fault & FaultType_FAULT) {
        append_reason(reason, "nFAULT asserted");
    }

    if (reason.empty()) {
        reason = "unknown DRV8301 fault";
    }

    return -2;
}

bool DRV8301::get_phrase_current(float& c1, float& c2, float& c3) {
    if (!is_ready()) {
        return false;
    }

    uint16_t raw_b = 0U;
    uint16_t raw_c = 0U;
    const uint64_t now = BSP::get_time_us();
    if (!measure_.get_raw_adc_value(BSP::Measure::PhraseB, raw_b, kSampleFreshnessUs, now) ||
        !measure_.get_raw_adc_value(BSP::Measure::PhraseC, raw_c, kSampleFreshnessUs, now)) {
        return false;
    }

    c2 = phase_current_from_adc(raw_b, current_offsets_[0]);
    c3 = phase_current_from_adc(raw_c, current_offsets_[1]);
    c1 = -(c2 + c3);
    return true;
}

bool DRV8301::get_vbus_voltage(float& vol) {
    uint16_t raw_vbus = 0U;
    if (!measure_.get_raw_adc_value(BSP::Measure::DCBusVol, raw_vbus, kSampleFreshnessUs, BSP::get_time_us())) {
        return false;
    }

    vol = static_cast<float>(raw_vbus) * kAdcReferenceVolts * kVbusDividerRatio / kAdcFullScale;
    return true;
}

bool DRV8301::set_phrase_voltage(float v1, float v2, float v3) {
    if (state_ != State::Ready || !enabled_) {
        svpwm_.set_voltage(24.0f, 0.0f, 0.0f, 0.0f);
        return false;
    }

    float vbus = 0.0f;
    if (!get_vbus_voltage(vbus) || !(vbus > 1e-3f)) {
        svpwm_.set_voltage(24.0f, 0.0f, 0.0f, 0.0f);
        return v1 == 0.0f && v2 == 0.0f && v3 == 0.0f;
    }

    svpwm_.set_voltage(vbus, v1, v2, v3);
    return true;
}

void DRV8301::do_checks() {
    if (state_ != State::Uninitialized && !fault_.read()) {
        last_fault_ |= FaultType_FAULT;
        state_ = State::Uninitialized;
        enabled_ = false;
        svpwm_.set_voltage(0.0f, 0.0f, 0.0f, 0.0f);
    }
}

DRV8301::FaultType_e DRV8301::get_error() {
    StatusRegisters status{};
    if (!read_status(status)) {
        return FaultType_SPI;
    }

    uint32_t fault = static_cast<uint32_t>(status.status1);
    fault |= (static_cast<uint32_t>(status.status2 & 0x0080U) << 16);
    last_fault_ = fault;
    return static_cast<FaultType_e>(fault);
}

bool DRV8301::read_status(StatusRegisters& status) {
    return read_reg(kRegNameStatus1, status.status1) &&
           read_reg(kRegNameStatus2, status.status2);
}

bool DRV8301::clear_faults() {
    const uint16_t gate_reset = static_cast<uint16_t>(regs_.control_register_1 | (1U << 2));
    bool ok = write_reg(kRegNameControl1, gate_reset) &&
              write_reg(kRegNameControl1, regs_.control_register_1);
    if (ok) {
        last_fault_ = FaultType_NoFault;
    }
    return ok;
}


bool DRV8301::calibrate_current_offsets(uint32_t samples, uint32_t timeout_ms) {
    if (samples == 0U) {
        return false;
    }

    uint32_t sum_b = 0U;
    uint32_t sum_c = 0U;
    uint32_t count = 0U;
    const uint32_t start_ms = HAL_GetTick();

    dc_cal_.write(true);
    delay_us(100);

    while (count < samples && (HAL_GetTick() - start_ms) <= timeout_ms) {
        uint16_t raw_b = 0U;
        uint16_t raw_c = 0U;
        const uint64_t now = BSP::get_time_us();
        if (measure_.get_raw_adc_value(BSP::Measure::PhraseB, raw_b, kSampleFreshnessUs, now) &&
            measure_.get_raw_adc_value(BSP::Measure::PhraseC, raw_c, kSampleFreshnessUs, now)) {
            sum_b += raw_b;
            sum_c += raw_c;
            ++count;
        }
        delay_us(100);
    }

    dc_cal_.write(false);

    if (count == 0U) {
        return false;
    }

    current_offsets_[0] = static_cast<uint16_t>(sum_b / count);
    current_offsets_[1] = static_cast<uint16_t>(sum_c / count);
    return count == samples;
}

uint16_t DRV8301::build_ctrl_word(CtrlMode_e ctrl_mode, RegName_e reg_name, uint16_t data) {
    return static_cast<uint16_t>(ctrl_mode | reg_name | (data & 0x07FFU));
}

bool DRV8301::read_reg(RegName_e reg_name, uint16_t& data) {
    const uint16_t command = build_ctrl_word(DRV8301_CtrlMode_Read, reg_name, 0U);
    uint16_t ignored = 0U;
    if (!transfer_word(command, &ignored)) {
        return false;
    }

    delay_us(1);

    uint16_t response = 0U;
    if (!transfer_word(command, &response)) {
        return false;
    }

    delay_us(1);

    data = static_cast<uint16_t>(response & 0x07FFU);
    return true;
}

bool DRV8301::write_reg(RegName_e reg_name, uint16_t data) {
    const uint16_t command = build_ctrl_word(DRV8301_CtrlMode_Write, reg_name, data);
    if (!transfer_word(command, nullptr)) {
        return false;
    }

    delay_us(1);
    return true;
}

bool DRV8301::transfer_word(uint16_t tx_word, uint16_t* rx_word) {
    tx_buf_[0] = static_cast<uint8_t>((tx_word >> 8) & 0xFFU);
    tx_buf_[1] = static_cast<uint8_t>(tx_word & 0xFFU);
    rx_buf_[0] = 0U;
    rx_buf_[1] = 0U;

    if (spi_.native() == nullptr) {
        return false;
    }

    BSP::ic_spi_cs().write(false);
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(spi_.native(), tx_buf_, rx_buf_, sizeof(tx_buf_), kSpiTimeoutMs);
    BSP::ic_spi_cs().write(true);
    if (status != HAL_OK) {
        return false;
    }

    if (rx_word) {
        *rx_word = static_cast<uint16_t>((static_cast<uint16_t>(rx_buf_[0]) << 8) | rx_buf_[1]);
    }
    return true;
}

float DRV8301::phase_current_from_adc(uint16_t raw, uint16_t offset) const {
    const int32_t adc_delta = static_cast<int32_t>(raw) - static_cast<int32_t>(offset);
    const float amp_out_volts = static_cast<float>(adc_delta) * kAdcReferenceVolts / kAdcFullScale;
    return amp_out_volts / (current_gain_ * kShuntResistanceOhms);
}
