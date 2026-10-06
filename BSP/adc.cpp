#include "adc.hpp"

#include <utility>

namespace bsp {

struct AdcChannel::State {
    ADC_HandleTypeDef* handle = nullptr;
    std::function<void()> dma_cplt;
    std::function<void()> injected_cplt;
};

AdcChannel::State AdcChannel::states_[AdcChannel::kMaxAdcCount]{};

AdcChannel::State* AdcChannel::findState(ADC_HandleTypeDef* handle)
{
    for (State& state : states_) {
        if (state.handle == handle) {
            return &state;
        }
    }
    return nullptr;
}

AdcChannel::State* AdcChannel::ensureState(ADC_HandleTypeDef* handle)
{
    if (handle == nullptr) {
        return nullptr;
    }

    State* state = findState(handle);
    if (state != nullptr) {
        return state;
    }

    for (State& empty_state : states_) {
        if (empty_state.handle == nullptr) {
            empty_state.handle = handle;
            return &empty_state;
        }
    }

    return nullptr;
}

void AdcChannel::dispatchDmaCplt(ADC_HandleTypeDef* handle)
{
    State* state = findState(handle);
    if (state != nullptr && state->dma_cplt) {
        state->dma_cplt();
    }
}

void AdcChannel::dispatchInjectedCplt(ADC_HandleTypeDef* handle)
{
    State* state = findState(handle);
    if (state != nullptr && state->injected_cplt) {
        state->injected_cplt();
    }
}

void AdcChannel::setDmaAddress(uint32_t* data)
{
    dma_address_ = data;
}

HAL_StatusTypeDef AdcChannel::startDma(uint32_t length, Callback cplt) const
{
    return startDma(nullptr, length, std::move(cplt));
}

HAL_StatusTypeDef AdcChannel::startDma(uint32_t* data, uint32_t length, Callback cplt) const
{
    uint32_t* dma_data = dma_address_ != nullptr ? dma_address_ : data;
    if (handle_ == nullptr || dma_data == nullptr || length == 0U) {
        return HAL_ERROR;
    }

    State* state = ensureState(handle_);
    if (state == nullptr) {
        return HAL_ERROR;
    }
    state->dma_cplt = std::move(cplt);

    if (HAL_ADC_RegisterCallback(handle_, HAL_ADC_CONVERSION_COMPLETE_CB_ID, &AdcChannel::dispatchDmaCplt) != HAL_OK) {
        state->dma_cplt = nullptr;
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_ADC_Start_DMA(handle_, dma_data, length);
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
        State* state = findState(handle_);
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

    State* state = ensureState(handle_);
    if (state == nullptr) {
        return HAL_ERROR;
    }
    state->injected_cplt = std::move(cplt);

    if (HAL_ADC_RegisterCallback(handle_, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, &AdcChannel::dispatchInjectedCplt) != HAL_OK) {
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
