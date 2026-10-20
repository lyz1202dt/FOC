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
    static AS5047P test_encoder(test_flash);
    BSP::sensor_spi_cs().write(true);
    as5047p_sample_result.spi_ready = BSP::sensor_spi_bus().init();
    if (as5047p_sample_result.spi_ready) {
        as5047p_sample_result.encoder_ready = test_encoder.init();
    }

    uint32_t led_count = 0U;
    while (true) {
        // get_value() 返回上次的位置，并发起一帧异步传输；前两帧建立读取流水线。
        if (as5047p_sample_result.encoder_ready) {
            as5047p_sample_result.angle_rad = test_encoder.get_value();
            as5047p_sample_result.value_available = test_encoder.value_is_available(10000U);
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
            as5600_test_result.angle_rad = test_encoder.get_value();
            as5600_test_result.value_available = test_encoder.value_is_available(10000U);
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
        test_flash, BSP::ic_spi_bus(), BSP::tim1_svpwm(), BSP::adc_measure(), BSP::drv8301_enable_gate(), BSP::drv8301_fault(),
        BSP::drv8301_dc_cal());
    BSP::drv8301_enable_gate().write(false);
    BSP::drv8301_dc_cal().write(false);
    BSP::ic_spi_cs().write(true);

    do {
        drv8301_sample_result.stage = "SpiInit";
        if (!BSP::ic_spi_bus().init()) {
            drv8301_sample_result.reason = "SPI bus initialization failed";
            break;
        }

        // 先使 ADC 等待外部触发，驱动器配置完成后再启动 TIM1 TRGO。
        drv8301_sample_result.stage = "AdcStart";
        if (!BSP::adc_measure_start()) {
            drv8301_sample_result.reason = "ADC injected sampling start failed";
            break;
        }
        // init() 仅执行 EN_GATE 上电、配置寄存器写入/回读及故障检查。
        drv8301_sample_result.stage = "DriverInit";
        if (!driver.init()) {
            driver.lookup_reason(drv8301_sample_result.reason);
            drv8301_sample_result.status_valid = driver.read_status(drv8301_sample_result.status);
            break;
        }

        drv8301_sample_result.stage = "PwmInit";
        if (!BSP::tim1_svpwm().init()) {
            drv8301_sample_result.reason = "TIM1 PWM initialization failed";
            break;
        }

        // 关闭 PWM 输出，保留 TIM1 运行以触发 ADC 采样。
        BSP::tim1_svpwm().enable_output(false);
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
        if (!driver.is_ready()) {
            driver.lookup_reason(drv8301_sample_result.reason);
        }
    }

    while (true) {
        if (drv8301_sample_result.passed) {
            drv8301_sample_result.status_valid = driver.read_status(drv8301_sample_result.status);
            const bool ready = driver.is_ready();
            const int reason_code = driver.lookup_reason(drv8301_sample_result.reason);
            if (!drv8301_sample_result.status_valid || !ready || reason_code != 0) {
                // 保留首个失败结果，不自动重启或清故障。
                if (!drv8301_sample_result.status_valid) {
                    drv8301_sample_result.reason = "Status register SPI read failed";
                }
                drv8301_sample_result.passed = false;
                drv8301_sample_result.stage = "MonitorFailed";
            }
        }

        if (!drv8301_sample_result.passed) {
            driver.set_enable(false);
            BSP::drv8301_dc_cal().write(false);
        }
        // 通过：每 500ms 翻转 LED；失败：每 100ms 翻转 LED。
        bsp::hardware::led.toggle();
        vTaskDelay(pdMS_TO_TICKS(drv8301_sample_result.passed ? 500 : 100));
    }

}
#endif  // TEST_DRV8301_ENABLED
