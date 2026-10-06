#pragma once

#include "stm32g4xx_hal.h"

#include <cstddef>
#include <functional>

namespace bsp {

class UartPort {
public:
    explicit constexpr UartPort(UART_HandleTypeDef& handle) : handle_(&handle) {}
    bool init();

    HAL_StatusTypeDef transmitIt(const void* data, uint16_t size,
                                 std::function<void(const void* data, uint16_t size)> finished_cb) const;
    HAL_StatusTypeDef receiveIt(void* data, uint16_t size,
                                std::function<void(const void* data, uint16_t size)> finished_cb) const;

    HAL_StatusTypeDef transmitDma(const void* data, uint16_t size,
                                  std::function<void(const void* data, uint16_t size)> finished_cb) const;
    HAL_StatusTypeDef receiveDma(void* data, uint16_t size,
                                 std::function<void(const void* data, uint16_t size)> finished_cb) const;
    HAL_StatusTypeDef receiveToIdleDma(void* data, uint16_t size,
                                       std::function<void(const void* data, uint16_t size)> finished_cb,
                                       bool disable_half_transfer_it = true) const;

    HAL_StatusTypeDef stopDma() const;
    HAL_StatusTypeDef abort() const;
    HAL_StatusTypeDef abortTransmit() const;
    HAL_StatusTypeDef abortReceive() const;

    UART_HandleTypeDef* native() const { return handle_; }

private:
    using FinishedCallback = std::function<void(const void*, uint16_t)>;

    struct TransferState;
    struct Slot;

    static constexpr std::size_t kMaxPortCount = 4U;
    static Slot slots_[kMaxPortCount];

    static void txHalfCpltCallback(UART_HandleTypeDef* huart);
    static void txCpltCallback(UART_HandleTypeDef* huart);
    static void rxHalfCpltCallback(UART_HandleTypeDef* huart);
    static void rxCpltCallback(UART_HandleTypeDef* huart);
    static void errorCallback(UART_HandleTypeDef* huart);
    static void abortCpltCallback(UART_HandleTypeDef* huart);
    static void abortTransmitCpltCallback(UART_HandleTypeDef* huart);
    static void abortReceiveCpltCallback(UART_HandleTypeDef* huart);
    static void rxEventCallback(UART_HandleTypeDef* huart, uint16_t size);

    static Slot* findSlot(UART_HandleTypeDef* huart);
    static Slot* ensureSlot(UART_HandleTypeDef* huart);
    static void clearTransmitState(TransferState& state);
    static void clearReceiveState(TransferState& state);
    static void completeTransmit(UART_HandleTypeDef* huart);
    static void completeReceive(UART_HandleTypeDef* huart, uint16_t size);
    static void clearAllStates(UART_HandleTypeDef* huart);

    UART_HandleTypeDef* handle_ = nullptr;
};

} // namespace bsp
