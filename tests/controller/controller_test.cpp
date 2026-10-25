#include "encoder_calib.hpp"
#include "voc.hpp"
#include "hardware.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>

static bool near(float a, float b) {
    return std::fabs(a - b) < 0.0001f;
}

int main() {
    Flash flash;
    EncoderCalib calib(flash);
    float a, b, c, offset = -99.0f;
    assert(!calib.get_offset(offset));
    assert(!calib.get_output(a, b, c));
    assert(a == 0.0f && b == 0.0f && c == 0.0f);

    // Start at time zero, then stop exactly at the two-second boundary.
    BSP::test_time_us = 0U;
    calib.set_measure(0, 0, 0, 24, 1.0f, true);
    assert(calib.get_output(a, b, c));
    assert(a == 3.0f && b == -1.5f && c == -1.5f);
    BSP::test_time_us = 1999999U;
    assert(calib.get_output(a, b, c) && a == 3.0f);
    assert(!calib.get_offset(offset));
    BSP::test_time_us = 2000000U;
    calib.set_measure(0, 0, 0, 24, 1.25f, true);
    assert(calib.get_output(a, b, c));
    assert(a == 0.0f && b == 0.0f && c == 0.0f);
    assert(calib.get_offset(offset) && offset == 1.25f);
    calib.set_measure(0, 0, 0, 24, 2.0f, true);
    BSP::test_time_us += 1000000U;
    assert(calib.get_output(a, b, c) && a == 0.0f);
    assert(calib.get_offset(offset) && offset == 1.25f);

    // Invalid data interrupts alignment and requires another full two seconds.
    calib.reset();
    assert(!calib.get_offset(offset));
    calib.set_measure(0, 0, 0, 24, 1.0f, true);
    assert(calib.get_output(a, b, c));
    BSP::test_time_us += 1000000U;
    calib.set_measure(0, 0, 0, 24, 1.0f, false);
    assert(!calib.get_output(a, b, c) && a == 0.0f);
    BSP::test_time_us += 1000000U;
    calib.set_measure(0, 0, 0, 24, 1.5f, true);
    assert(calib.get_output(a, b, c) && a == 3.0f);
    BSP::test_time_us += 1999999U;
    assert(calib.get_output(a, b, c) && a == 3.0f);
    BSP::test_time_us += 1U;
    assert(calib.get_output(a, b, c) && a == 0.0f);
    assert(calib.get_offset(offset) && offset == 1.5f);
    calib.reset();
    calib.set_measure(0, 0, 0, 24, std::numeric_limits<float>::quiet_NaN(), true);
    assert(!calib.get_output(a, b, c));
    assert(flash.reads == 0U);  // No Flash access from ISR controller callbacks.

    VOC voc(flash);
    assert(voc.load_config());
    assert(test_interrupt_mask == 0U);
    voc.reset();
    // The calibrated mechanical position must map to zero electrical angle.
    voc.set_measure(0, 0, 0, 24, flash.offset, true);
    assert(voc.get_output(a, b, c));
    assert(near(a, 0.0f) && near(b, 0.866025404f) && near(c, -0.866025404f));
    constexpr float kHalfPi = 1.57079632679489661923f;
    for (int n : {1, 7, 14}) {
        flash.n = n;
        flash.offset = 6.2f;
        test_interrupt_mask = 1U;
        assert(voc.load_config() && test_interrupt_mask == 1U);
        test_interrupt_mask = 0U;
        voc.reset();
        const float rad = std::fmod(flash.offset + kHalfPi / n, 6.28318530717958647692f);
        voc.set_measure(0, 0, 0, 24, rad, true);
        assert(voc.get_output(a, b, c));
        assert(near(a, -1.0f) && near(b, 0.5f) && near(c, 0.5f));
        const unsigned reads = flash.reads;
        voc.set_measure(0, 0, 0, 24, rad, true);
        assert(voc.get_output(a, b, c) && flash.reads == reads);
    }
    flash.n = 0;
    assert(!voc.load_config());
    flash.n = -1;
    assert(!voc.load_config());
    flash.n = 7;
    flash.offset = std::numeric_limits<float>::infinity();
    assert(!voc.load_config());
    flash.offset = 0.0f;
    flash.read_ok = false;
    assert(!voc.load_config());
    voc.controller_reference(nullptr, 0U);
    std::puts("Controller tests passed: alignment timing, invalid samples, mechanical offset, pole pairs, config reload.");
}
