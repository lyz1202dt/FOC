#include "tim.hpp"

#include <utility>

namespace bsp {

struct Timer::CallbackSlot {
    TIM_HandleTypeDef* handle = nullptr;
    bsp::Timer::PeriodElapsedCallback callback;
};

Timer::CallbackSlot Timer::callback_slots_[Timer::kMaxCallbackCount]{};

Timer::CallbackSlot* Timer::findSlot(TIM_HandleTypeDef* handle)
{
    for (CallbackSlot& slot : callback_slots_) {
        if (slot.handle == handle) {
            return &slot;
        }
    }
    return nullptr;
}

Timer::CallbackSlot* Timer::ensureSlot(TIM_HandleTypeDef* handle)
{
    if (handle == nullptr) {
        return nullptr;
    }

    CallbackSlot* slot = findSlot(handle);
    if (slot != nullptr) {
        return slot;
    }

    for (CallbackSlot& empty_slot : callback_slots_) {
        if (empty_slot.handle == nullptr) {
            empty_slot.handle = handle;
            return &empty_slot;
        }
    }

    return nullptr;
}

void Timer::periodElapsedCallback(TIM_HandleTypeDef* handle)
{
    CallbackSlot* slot = findSlot(handle);
    if (slot != nullptr && slot->callback) {
        slot->callback();
    }
}

HAL_StatusTypeDef Timer::registerPeriodElapsedCallback(PeriodElapsedCallback callback) const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    CallbackSlot* slot = ensureSlot(handle_);
    if (slot == nullptr) {
        return HAL_ERROR;
    }

    slot->callback = std::move(callback);
    return HAL_TIM_RegisterCallback(handle_, HAL_TIM_PERIOD_ELAPSED_CB_ID, &Timer::periodElapsedCallback);
}

HAL_StatusTypeDef Timer::start() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_Base_Start(handle_);
}

HAL_StatusTypeDef Timer::stop() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_Base_Stop(handle_);
}

HAL_StatusTypeDef Timer::startIt() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_Base_Start_IT(handle_);
}

HAL_StatusTypeDef Timer::stopIt() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_Base_Stop_IT(handle_);
}

void Timer::setCounter(uint32_t counter) const
{
    if (handle_ != nullptr) {
        __HAL_TIM_SET_COUNTER(handle_, counter);
    }
}

uint32_t Timer::counter() const
{
    if (handle_ == nullptr) {
        return 0U;
    }
    return __HAL_TIM_GET_COUNTER(handle_);
}

void Timer::setAutoreload(uint32_t autoreload) const
{
    if (handle_ != nullptr) {
        __HAL_TIM_SET_AUTORELOAD(handle_, autoreload);
    }
}

uint32_t Timer::autoreload() const
{
    if (handle_ == nullptr) {
        return 0U;
    }
    return __HAL_TIM_GET_AUTORELOAD(handle_);
}

HAL_StatusTypeDef PwmChannel::start() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Start(handle_, channel_);
}

HAL_StatusTypeDef PwmChannel::stop() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Stop(handle_, channel_);
}

HAL_StatusTypeDef PwmChannel::startComplementary() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIMEx_PWMN_Start(handle_, channel_);
}

HAL_StatusTypeDef PwmChannel::stopComplementary() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIMEx_PWMN_Stop(handle_, channel_);
}

HAL_StatusTypeDef PwmChannel::startIt() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Start_IT(handle_, channel_);
}

HAL_StatusTypeDef PwmChannel::stopIt() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Stop_IT(handle_, channel_);
}

void PwmChannel::setDmaAddress(const uint32_t* data)
{
    dma_address_ = data;
}

HAL_StatusTypeDef PwmChannel::startDma(uint16_t length) const
{
    return startDma(nullptr, length);
}

HAL_StatusTypeDef PwmChannel::startDma(const uint32_t* data, uint16_t length) const
{
    const uint32_t* dma_data = dma_address_ != nullptr ? dma_address_ : data;
    if (handle_ == nullptr || dma_data == nullptr || length == 0U) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Start_DMA(handle_, channel_, dma_data, length);
}

HAL_StatusTypeDef PwmChannel::stopDma() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Stop_DMA(handle_, channel_);
}

void PwmChannel::setPulse(uint32_t pulse) const
{
    if (handle_ != nullptr) {
        __HAL_TIM_SET_COMPARE(handle_, channel_, pulse);
    }
}

uint32_t PwmChannel::pulse() const
{
    if (handle_ == nullptr) {
        return 0U;
    }
    return __HAL_TIM_GET_COMPARE(handle_, channel_);
}

void PwmChannel::setDutyPermille(uint16_t duty_permille) const
{
    if (handle_ == nullptr) {
        return;
    }

    if (duty_permille > 1000U) {
        duty_permille = 1000U;
    }

    const uint32_t compare = (__HAL_TIM_GET_AUTORELOAD(handle_) * duty_permille) / 1000U;
    setPulse(compare);
}

} // namespace bsp
