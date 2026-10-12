#pragma once

#include "stm32g4xx_hal.h"

#include <cstddef>
#include <functional>

namespace bsp {

class Timer {
public:
    using PeriodElapsedCallback = std::function<void()>;

    explicit constexpr Timer(TIM_HandleTypeDef& handle) : handle_(&handle) {}

    HAL_StatusTypeDef registerPeriodElapsedCallback(PeriodElapsedCallback callback) const;

    HAL_StatusTypeDef start() const;
    HAL_StatusTypeDef stop() const;
    HAL_StatusTypeDef startIt() const;
    HAL_StatusTypeDef stopIt() const;

    void setCounter(uint32_t counter) const;
    uint32_t counter() const;
    void setAutoreload(uint32_t autoreload) const;
    uint32_t autoreload() const;

    TIM_HandleTypeDef* native() const { return handle_; }

private:
    struct CallbackSlot;

    static constexpr std::size_t kMaxCallbackCount = 6U;
    static CallbackSlot callback_slots_[kMaxCallbackCount];

    static CallbackSlot* findSlot(TIM_HandleTypeDef* handle);
    static CallbackSlot* ensureSlot(TIM_HandleTypeDef* handle);
    static void periodElapsedCallback(TIM_HandleTypeDef* handle);

    TIM_HandleTypeDef* handle_ = nullptr;
};

class PwmChannel {
public:
    constexpr PwmChannel() = default;
    constexpr PwmChannel(TIM_HandleTypeDef& handle, uint32_t channel)
        : handle_(&handle), channel_(channel)
    {
    }

    TIM_HandleTypeDef* native() const { return handle_; }

    HAL_StatusTypeDef start() const;
    HAL_StatusTypeDef stop() const;
    HAL_StatusTypeDef startComplementary() const;
    HAL_StatusTypeDef stopComplementary() const;
    HAL_StatusTypeDef startIt() const;
    HAL_StatusTypeDef stopIt() const;
    void setDmaAddress(const uint32_t* data);
    const uint32_t* dmaAddress() const { return dma_address_; }
    HAL_StatusTypeDef startDma(uint16_t length) const;
    HAL_StatusTypeDef startDma(const uint32_t* data, uint16_t length) const;
    HAL_StatusTypeDef stopDma() const;

    void setPulse(uint32_t pulse) const;
    uint32_t pulse() const;
    void setDutyPermille(uint16_t duty_permille) const;

private:
    TIM_HandleTypeDef* handle_ = nullptr;
    uint32_t channel_ = TIM_CHANNEL_1;
    const uint32_t* dma_address_ = nullptr;
};

} // namespace bsp
