#include "test.hpp"

#if TEST_AS5047P_ENABLED || TEST_AS5600_ENABLED || TEST_DRV8301_ENABLED
#include "FreeRTOS.h"
#include "flash.hpp"
#include "hardware.hpp"
#include "task.h"
#endif

#if TEST_AS5047P_ENABLED
#include "as5047p.hpp"

// 结果仅由调用测试的任务写入；暂停 CPU 后通过调试器查看。
struct As5047pTestResult {
    bool spi_ready{false};
    bool encoder_ready{false};
    bool value_available{false};
    float angle_rad{0.0f};
};

As5047pTestResult as5047p_sample_result;

[[noreturn]] void as5047p_test() {
    BSP::time_init();
    static Flash test_flash;
    test_flash.init();

    // AS5047P 独立采样测试：不启动电机控制环，默认使用 SPI IT 模式。
    static AS5047P test_encoder(test_flash, BSP::sensor_spi_bus(), BSP::sensor_spi_cs());
    BSP::sensor_spi_cs().write(true);
    as5047p_sample_result.spi_ready = BSP::sensor_spi_bus().init();
    if (as5047p_sample_result.spi_ready) {
        as5047p_sample_result.encoder_ready = test_encoder.init();
    }

    uint32_t led_count = 0U;
    while (true) {
        // get_value() 输出缓存位置、返回新鲜度，并发起异步传输；前两帧建立读取流水线。
        if (as5047p_sample_result.encoder_ready) {
            as5047p_sample_result.value_available = test_encoder.get_value(as5047p_sample_result.angle_rad, 10000U);
        }
        // 新鲜数据：约 500ms 翻转 LED；初始化失败或数据超时：约 100ms。
        const uint32_t led_period = as5047p_sample_result.value_available ? 500U : 100U;
        if (++led_count >= led_period) {
            led_count = 0U;
            bsp::hardware::led.toggle();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

}
#endif  // TEST_AS5047P_ENABLED

#if TEST_AS5600_ENABLED
#include "as5600.hpp"

// 结果仅由调用测试的任务写入；暂停 CPU 后通过调试器查看。
struct As5600TestResult {
    bool i2c_ready{false};
    bool encoder_ready{false};
    bool value_available{false};
    bool magnet_status_valid{false};
    AS5600::MagnetStatus magnet_status{};
    float angle_rad{0.0f};
};

As5600TestResult as5600_test_result;

[[noreturn]] void as5600_test() {
    BSP::time_init();
    static Flash test_flash;
    test_flash.init();

    static AS5600 test_encoder(test_flash, BSP::sensor_i2c_bus());
    as5600_test_result.i2c_ready = BSP::sensor_i2c_bus().init();
    if (as5600_test_result.i2c_ready) {
        as5600_test_result.encoder_ready = test_encoder.init();
    }

    uint32_t led_count = 0U;
    while (true) {
        if (as5600_test_result.encoder_ready) {
            as5600_test_result.value_available = test_encoder.get_value(as5600_test_result.angle_rad, 10000U);
            as5600_test_result.magnet_status_valid = test_encoder.GetMagnetStatus(as5600_test_result.magnet_status);
        }
        const uint32_t led_period = as5600_test_result.value_available ? 500U : 100U;
        if (++led_count >= led_period) {
            led_count = 0U;
            bsp::hardware::led.toggle();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif  // TEST_AS5600_ENABLED

#if TEST_DRV8301_ENABLED
#include "drv8301.hpp"

#include <string>

// 结果仅由调用测试的任务写入；暂停 CPU 后通过调试器查看。
struct Drv8301TestResult {
    const char* stage{"NotStarted"};
    bool passed{false};
    bool status_valid{false};
    DRV8301::StatusRegisters status{};
    DRV8301::FaultType_e fault{DRV8301::FaultType_NoFault};
    std::string reason;
    float vbus_voltage{0.0f};
    float phase_a_current{0.0f};
    float phase_b_current{0.0f};
    float phase_c_current{0.0f};
};

Drv8301TestResult drv8301_sample_result;

[[noreturn]] void drv8301_test() {
    BSP::time_init();
    static Flash test_flash;
    test_flash.init();

    static DRV8301 driver(
        test_flash, BSP::ic_spi_bus(), BSP::ic_spi_cs(), BSP::tim1_svpwm(), BSP::adc_measure(),
        BSP::drv8301_enable_gate(), BSP::drv8301_fault(),
        BSP::drv8301_dc_cal());
    do {
        drv8301_sample_result.stage = "SpiInit";
        if (!BSP::ic_spi_bus().init()) {
            drv8301_sample_result.reason = "SPI bus initialization failed";
            break;
        }

        // init() 配置芯片和 PWM 外设，保持功率输出关闭。
        drv8301_sample_result.stage = "DriverInit";
        if (!driver.init()) {
            drv8301_sample_result.reason = "Driver initialization failed";
            drv8301_sample_result.status_valid = driver.read_status(drv8301_sample_result.status);
            break;
        }

        drv8301_sample_result.stage = "MeasureStart";
        if (!driver.enable_measure(true)) {
            drv8301_sample_result.reason = "ADC/TIM1 sampling start failed";
            break;
        }

        // 关闭 PWM 输出，保留 TIM1 运行以触发 ADC 采样。
        driver.enable_pwm(false);
        drv8301_sample_result.stage = "CurrentCalibration";
        if (!driver.self_check()) {
            drv8301_sample_result.reason = "Current offset calibration failed";
            break;
        }
        // 仅测试通讯与 ADC，保持 PWM 输出关闭。

        drv8301_sample_result.stage = "AdcSamples";
        if (!driver.get_vbus_voltage(drv8301_sample_result.vbus_voltage) ||
            !driver.get_phrase_current(drv8301_sample_result.phase_a_current,
                                       drv8301_sample_result.phase_b_current,
                                       drv8301_sample_result.phase_c_current)) {
            drv8301_sample_result.reason = "Fresh voltage/current samples unavailable";
            break;
        }
        drv8301_sample_result.passed = true;
        drv8301_sample_result.stage = "Monitoring";
    } while (false);

    if (!drv8301_sample_result.passed) {
        // 关闭 EN_GATE 前保留首次故障，避免复位芯片后丢失原因。
        drv8301_sample_result.status_valid = driver.read_status(drv8301_sample_result.status);
        drv8301_sample_result.fault = driver.get_error();
    }

    while (true) {
        if (drv8301_sample_result.passed) {
            drv8301_sample_result.status_valid = driver.read_status(drv8301_sample_result.status);
            const bool ready = driver.is_ready();
            drv8301_sample_result.fault = driver.get_error();
            const bool fault_active = !BSP::drv8301_fault().read();
            if (!drv8301_sample_result.status_valid || !ready ||
                drv8301_sample_result.fault != DRV8301::FaultType_NoFault || fault_active) {
                // 保留首个失败结果，不自动重启或清故障。
                if (!drv8301_sample_result.status_valid)  {
                    drv8301_sample_result.reason = "Status register SPI read failed";
                } else if (!ready) {
                    drv8301_sample_result.reason = "Driver not ready";
                } else {
                    drv8301_sample_result.reason = "Driver fault detected";
                }
                drv8301_sample_result.passed = false;
                drv8301_sample_result.stage = "MonitorFailed";
            }
        }

        if (!drv8301_sample_result.passed) {
            driver.enable_pwm(false);
            driver.enable_measure(false);
            BSP::drv8301_dc_cal().write(false);
        }
        // 通过：每 500ms 翻转 LED；失败：每 100ms 翻转 LED。
        bsp::hardware::led.toggle();
        vTaskDelay(pdMS_TO_TICKS(drv8301_sample_result.passed ? 500 : 100));
    }

}
#endif  // TEST_DRV8301_ENABLED
