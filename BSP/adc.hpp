#pragma once

#include "stm32g4xx_hal.h"

#include <functional>

namespace bsp {

class AdcChannel {
public:
    using Callback = std::function<void()>;

    constexpr AdcChannel() = default;
    explicit constexpr AdcChannel(ADC_HandleTypeDef& handle) : handle_(&handle) {}

    HAL_StatusTypeDef startDma(uint32_t* data, uint32_t length, Callback cplt = nullptr) const;
    HAL_StatusTypeDef stopDma() const;

    HAL_StatusTypeDef injectedStartIt(Callback cplt = nullptr) const;
    HAL_StatusTypeDef injectedStopIt() const;
    uint32_t injectedValue(uint32_t rank) const;

    ADC_HandleTypeDef* native() const { return handle_; }

private:
    ADC_HandleTypeDef* handle_ = nullptr;
};

} // namespace bsp
