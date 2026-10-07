#pragma once

#include "stm32g4xx_hal.h"

#include <cstddef>
#include <functional>

namespace bsp {

class AdcChannel {
public:
    using Callback = std::function<void()>;

    constexpr AdcChannel() = default;
    explicit constexpr AdcChannel(ADC_HandleTypeDef& handle) : handle_(&handle) {}

    void setDmaAddress(uint32_t* data);
    uint32_t* dmaAddress() const { return dma_address_; }

    HAL_StatusTypeDef startDma(uint32_t length, Callback cplt = nullptr) const;
    HAL_StatusTypeDef stopDma() const;

    HAL_StatusTypeDef injectedStartIt(Callback cplt = nullptr) const;
    HAL_StatusTypeDef injectedStopIt() const;
    uint32_t injectedValue(uint32_t rank) const;

    ADC_HandleTypeDef* native() const { return handle_; }

private:
    struct State;

    static constexpr std::size_t kMaxAdcCount = 4U;
    static State states_[kMaxAdcCount];

    static State* findState(ADC_HandleTypeDef* handle);
    static State* ensureState(ADC_HandleTypeDef* handle);
    static void dispatchDmaCplt(ADC_HandleTypeDef* handle);
    static void dispatchInjectedCplt(ADC_HandleTypeDef* handle);

    ADC_HandleTypeDef* handle_ = nullptr;
    uint32_t* dma_address_ = nullptr;
};

} // namespace bsp
