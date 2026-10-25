#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>

#include "stm32g4xx.h"

template <typename T>
struct SampleItem {
    T data;
    uint64_t tick;
};

// 单核 Cortex-M 上供任务与普通 ISR 共享；不支持 NMI/HardFault 并发访问。
// 每个元素独立保持 data/tick 一致，不提供整组快照。
// T 应为体积较小的值类型，指针指向的数据不在保护范围内。
template <typename T, int N>
class SampleGroup {
    static_assert(N > 0, "SampleGroup must contain at least one item");
    static_assert(std::is_trivially_copyable<T>::value, "SampleGroup requires trivially copyable sample data");

public:
    // 在发布给任务/ISR 之前构造，初始数据与时间戳均为零初始化值。
    SampleGroup() = default;
    SampleGroup(const SampleGroup&) = delete;
    SampleGroup& operator=(const SampleGroup&) = delete;

    bool save(int index, const T& data, uint64_t tick) {
        if (index < 0 || index >= N) {
            return false;
        }

        const uint32_t interrupt_mask = __get_PRIMASK();
        __disable_irq();
        __DMB();
        std::memcpy(&items[index].data, &data, sizeof(T));
        items[index].tick = tick;
        __DMB();
        __set_PRIMASK(interrupt_mask);
        return true;
    }

    // 返回对应数据的时间戳；未采样或索引越界时返回 0。
    // 索引越界时不修改 data；合法时间戳 0 与未采样无法区分。
    uint64_t get(int index, T& data) const {
        if (index < 0 || index >= N) {
            return 0;
        }

        const uint32_t interrupt_mask = __get_PRIMASK();
        __disable_irq();
        __DMB();
        std::memcpy(&data, &items[index].data, sizeof(T));
        const uint64_t tick = items[index].tick;
        __DMB();
        __set_PRIMASK(interrupt_mask);
        return tick;
    }

private:
    SampleItem<T> items[N]{};
};
