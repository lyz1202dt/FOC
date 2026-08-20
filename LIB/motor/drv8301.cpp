#include "drv8301.hpp"

#include "adc.h"
#include "gpio.h"
#include "tim.h"

namespace {

constexpr uint32_t kSampleFreshnessUs = 10000U;
constexpr float kDefaultVbusVolts = 24.0f;
constexpr float kDefaultVbusCounts = 2048.0f;
GPIO_TypeDef* const kEnableGatePort = GPIOC;
constexpr uint16_t kEnableGatePin = GPIO_PIN_15;

} // namespace

DRV8301::DRV8301() {
    phase_current_measure_.load_config();
    svpwm_.load_config();
}

bool DRV8301::init() {
    ready_ = false;
    enabled_ = false;

    phase_current_measure_.load_config();
    svpwm_.load_config();

    BSP::bind_irq_func(BSP::ADC1_INJEC_IRQ,
                       [](void* ctx) {
                           auto* self = static_cast<DRV8301*>(ctx);
                           if (!self) {
                               return;
                           }
                           self->set_raw_adc_value(MotorDriver::PhraseB, hadc1.Instance->JDR1);
                           self->set_raw_adc_value(MotorDriver::PhraseC, hadc1.Instance->JDR2);
                           self->set_raw_adc_value(MotorDriver::DCBusVol, hadc1.Instance->JDR3);
                       },
                       this);

    set_enable(false);
    set_phrase_voltage(0.0f, 0.0f, 0.0f);

    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
        return false;
    }

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
        return false;
    }

    if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
        return false;
    }

    ready_ = true;
    return true;
}

bool DRV8301::is_ready() {
    return ready_ && enabled_;
}

bool DRV8301::set_enable(bool enable) {
    enabled_ = enable;
    HAL_GPIO_WritePin(kEnableGatePort, kEnableGatePin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (!enable) {
        set_phrase_voltage(0.0f, 0.0f, 0.0f);
    }
    return true;
}

int DRV8301::lookup_reason(std::string& reason) {
    if (!ready_) {
        reason = "DRV8301 not initialized";
        return -1;
    }
    if (!enabled_) {
        reason = "DRV8301 disabled";
        return 1;
    }
    if (HAL_GPIO_ReadPin(IC_FAULT_GPIO_Port, IC_FAULT_Pin) == GPIO_PIN_RESET) {
        reason = "DRV8301 fault asserted";
        return -2;
    }
    reason = "OK";
    return 0;
}

bool DRV8301::get_vbus_voltage(float& vol) {
    if (!adc_sample_fresh(DCBusVol, kSampleFreshnessUs)) {
        vol = kDefaultVbusVolts;
        return true;
    }

    vol = static_cast<float>(ADCRawValue[DCBusVol].value) * (kDefaultVbusVolts / kDefaultVbusCounts);
    return true;
}

bool DRV8301::get_phrase_current(float& c1, float& c2, float& c3) {
    if (!adc_sample_fresh(PhraseB, kSampleFreshnessUs) ||
        !adc_sample_fresh(PhraseC, kSampleFreshnessUs)) {
        c1 = 0.0f;
        c2 = 0.0f;
        c3 = 0.0f;
        return false;
    }

    phase_current_measure_.record_adc_value(ADCRawValue[PhraseB].value, ADCRawValue[PhraseC].value);
    phase_current_measure_.calc_current(c1, c2, c3);
    return true;
}

bool DRV8301::set_phrase_voltage(float v1, float v2, float v3) {
    float vbus = 0.0f;
    get_vbus_voltage(vbus);
    svpwm_.set_voltage(vbus, v1, v2, v3);
    return true;
}

bool DRV8301::adc_sample_fresh(ADCMotor channel, uint32_t max_age_us) const {
    const uint64_t sample_time = ADCRawValue[channel].time;
    if (sample_time == 0U) {
        return false;
    }
    return (BSP::get_time_us() - sample_time) <= max_age_us;
}
