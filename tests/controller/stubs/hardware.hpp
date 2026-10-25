#pragma once
#include <cstdint>
inline uint32_t test_interrupt_mask = 0U;
inline uint32_t __get_PRIMASK() { return test_interrupt_mask; }
inline void __disable_irq() { test_interrupt_mask = 1U; }
inline void __set_PRIMASK(uint32_t mask) { test_interrupt_mask = mask; }
namespace BSP {
inline uint64_t test_time_us = 0U;
inline uint64_t get_time_us() { return test_time_us; }
}
