#include "adc.hpp"

#include <cstddef>
#include <utility>

namespace {

struct AdcState {
    ADC_HandleTypeDef* handle = nullptr;
    std::function<void()> dma_cplt;
    std::function<void()> injected_cplt;
};

constexpr std::size_t kMaxAdcCount = 4U;
AdcState g_adc_states[kMaxAdcCount]{};

AdcState* findState(ADC_HandleTypeDef* handle)
{
    for (AdcState& state : g_adc_states) {
        if (state.handle == handle) {
            return &state;
        }
    }
    return nullptr;
}

AdcState* ensureState(ADC_HandleTypeDef* handle)
{
    if (handle == nullptr) {
        return nullptr;
    }

    AdcState* state = findState(handle);
    if (state != nullptr) {
        return state;
    }

    for (AdcState& empty_state : g_adc_states) {
        if (empty_state.handle == nullptr) {
            empty_state.handle = handle;
            return &empty_state;
        }
    }

    return nullptr;
}

void dispatchDmaCplt(ADC_HandleTypeDef* handle)
{
    AdcState* state = findState(handle);
    if (state != nullptr && state->dma_cplt) {
        state->dma_cplt();
    }
}

void dispatchInjectedCplt(ADC_HandleTypeDef* handle)
{
    AdcState* state = findState(handle);
    if (state != nullptr && state->injected_cplt) {
        state->injected_cplt();
    }
}

} // namespace

namespace bsp {

HAL_StatusTypeDef AdcChannel::startDma(uint32_t* data, uint32_t length, Callback cplt) const
{
    if (handle_ == nullptr || data == nullptr || length == 0U) {
        return HAL_ERROR;
    }

    AdcState* state = ensureState(handle_);
    if (state == nullptr) {
        return HAL_ERROR;
    }
    state->dma_cplt = std::move(cplt);

    if (HAL_ADC_RegisterCallback(handle_, HAL_ADC_CONVERSION_COMPLETE_CB_ID, &dispatchDmaCplt) != HAL_OK) {
        state->dma_cplt = nullptr;
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_ADC_Start_DMA(handle_, data, length);
    if (status != HAL_OK) {
        state->dma_cplt = nullptr;
    }
    return status;
}

HAL_StatusTypeDef AdcChannel::stopDma() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_ADC_Stop_DMA(handle_);
    if (status == HAL_OK) {
        AdcState* state = findState(handle_);
        if (state != nullptr) {
            state->dma_cplt = nullptr;
        }
    }
    return status;
}

HAL_StatusTypeDef AdcChannel::injectedStartIt(Callback cplt) const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    AdcState* state = ensureState(handle_);
    if (state == nullptr) {
        return HAL_ERROR;
    }
    state->injected_cplt = std::move(cplt);

    if (HAL_ADC_RegisterCallback(handle_, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, &dispatchInjectedCplt) != HAL_OK) {
        state->injected_cplt = nullptr;
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_ADCEx_InjectedStart_IT(handle_);
    if (status != HAL_OK) {
        state->injected_cplt = nullptr;
    }
    return status;
}

HAL_StatusTypeDef AdcChannel::injectedStopIt() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_ADCEx_InjectedStop_IT(handle_);
}

uint32_t AdcChannel::injectedValue(uint32_t rank) const
{
    if (handle_ == nullptr) {
        return 0U;
    }
    return HAL_ADCEx_InjectedGetValue(handle_, rank);
}

} // namespace bsp
