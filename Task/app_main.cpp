#include "app_main.hpp"
#include "as5047p.hpp"
#include "as5600.hpp"
#include "comm.hpp"
#include "drv8301.hpp"
#include "encoder.hpp"
#include "hardware.hpp"
#include "i2c.hpp"
#include "pack.h"
#include "projdefs.h"
#include "spi.hpp"
#include "svpwm.hpp"


#include "comm.hpp"
#include "flash.hpp"
#include "motor.hpp"
#include "uartcomm.hpp"

#include <atomic>
#include <string>

TaskHandle_t motor_slow_loop_task_handle;
void motor_slow_loop(void* param);

// 存储配置数据
static Flash flash;
void load_components_from_config(Encoder*& encoder, MotorDriver*& driver);

// 组装电机控制器
CommManager comm_manager(2);
Motor motor(flash, comm_manager);

UartComm uart_comm(BSP::uart_comm_port());

// 仅 app_main 线程写入；暂停 CPU 后在调试器中查看，不供 ISR/其他任务读取。
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

Drv8301TestResult drv8301_test_result;

// uint8_t send_buffer[16]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
// uint8_t send_buffer2[16]={15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0};

// float debug_rad;
extern "C" void app_main(void* param) {
    // Command* send1;
    // Command* send2;
    // send1=(Command*)send_buffer;
    // send2=(Command*)send_buffer2;
    // send1->size=8;
    // send2->size=8;
    // bsp::hardware::spi1.init();
    // bsp::hardware::spi3.init();
    // bsp::hardware::i2c1.init();

    BSP::time_init();
    // BSP::adc_measure_start();
    // BSP::ic_spi_bus().init();
    // BSP::tim1_svpwm().init();
    // BSP::sensor_spi_bus().init();


    flash.init();

    // Encoder* encoder = nullptr;
    // MotorDriver* driver = nullptr;
    // load_components_from_config(encoder, driver);
    // if (!motor.install_driver(encoder, driver)) {
    //     while (1) {
    //         vTaskDelay(pdMS_TO_TICKS(1000));
    //     }
    // }
    // const bool i2c_ready = BSP::sensor_i2c_bus().init();
    // Encoder* encoder = new AS5600(flash, BSP::sensor_i2c_bus());
    // encoder->init();

    // int cnt=0;
    // while (1) {
    //     cnt=(cnt+1)%500;
    //     if(!cnt)
    //         bsp::hardware::led.toggle();
    //     debug_rad=encoder->get_value();
    //     vTaskDelay(pdMS_TO_TICKS(1));
    // }

    // uart_comm.init();
    // uart_comm.register_recv_callback([](Command* data){
    //     bsp::hardware::led.toggle();
    // });
    // while(1)
    // {
    //     uart_comm.send_pack(send1);
    //     uart_comm.send_pack(send2);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    // comm_manager.register_comm_channel(&uart_comm);
    // comm_manager.register_comm_channel(&uart_comm); //其它的通信接口
    // 上电通讯测试：先配置驱动器，最后启动 TIM1，不运行电机控制环。
    // 静态存储避免占用 defaultTask 的小栈，也不需要 new。
    static DRV8301 driver(
        flash, BSP::ic_spi_bus(), BSP::tim1_svpwm(), BSP::adc_measure(), BSP::drv8301_enable_gate(), BSP::drv8301_fault(),
        BSP::drv8301_dc_cal());
    BSP::drv8301_enable_gate().write(false);
    BSP::drv8301_dc_cal().write(false);
    BSP::ic_spi_cs().write(true);

    do {
        drv8301_test_result.stage = "SpiInit";
        if (!BSP::ic_spi_bus().init()) {
            drv8301_test_result.reason = "SPI bus initialization failed";
            break;
        }

        // 先使 ADC 等待外部触发，驱动器配置完成后再启动 TIM1 TRGO。
        drv8301_test_result.stage = "AdcStart";
        if (!BSP::adc_measure_start()) {
            drv8301_test_result.reason = "ADC injected sampling start failed";
            break;
        }
        // init() 仅执行 EN_GATE 上电、配置寄存器写入/回读及故障检查。
        drv8301_test_result.stage = "DriverInit";
        if (!driver.init()) {
            driver.lookup_reason(drv8301_test_result.reason);
            drv8301_test_result.status_valid = driver.read_status(drv8301_test_result.status);
            break;
        }

        drv8301_test_result.stage = "PwmInit";
        if (!BSP::tim1_svpwm().init()) {
            drv8301_test_result.reason = "TIM1 PWM initialization failed";
            break;
        }

        // 关闭 PWM 输出，保留 TIM1 运行以触发 ADC 采样。
        BSP::tim1_svpwm().enable_output(false);
        drv8301_test_result.stage = "CurrentCalibration";
        if (!driver.calibrate_current_offsets()) {
            drv8301_test_result.reason = "Current offset calibration failed";
            break;
        }
        BSP::tim1_svpwm().enable_output(true);

        drv8301_test_result.stage = "AdcSamples";
        if (!driver.get_vbus_voltage(drv8301_test_result.vbus_voltage) ||
            !driver.get_phrase_current(drv8301_test_result.phase_a_current,
                                       drv8301_test_result.phase_b_current,
                                       drv8301_test_result.phase_c_current)) {
            drv8301_test_result.reason = "Fresh voltage/current samples unavailable";
            break;
        }
        drv8301_test_result.passed = true;
        drv8301_test_result.stage = "Monitoring";
    } while (false);

    if (!drv8301_test_result.passed) {
        // 关闭 EN_GATE 前保留首次故障，避免复位芯片后丢失原因。
        drv8301_test_result.status_valid = driver.read_status(drv8301_test_result.status);
        if (!driver.is_ready()) {
            driver.lookup_reason(drv8301_test_result.reason);
        }
    }

    while (true) {
        if (drv8301_test_result.passed) {
            drv8301_test_result.status_valid = driver.read_status(drv8301_test_result.status);
            const bool ready = driver.is_ready();
            const int reason_code = driver.lookup_reason(drv8301_test_result.reason);
            if (!drv8301_test_result.status_valid || !ready || reason_code != 0) {
                // 保留首个失败结果，不自动重启或清故障。
                if (!drv8301_test_result.status_valid) {
                    drv8301_test_result.reason = "Status register SPI read failed";
                }
                drv8301_test_result.passed = false;
                drv8301_test_result.stage = "MonitorFailed";
            }
        }

        if (!drv8301_test_result.passed) {
            driver.set_enable(false);
            BSP::drv8301_dc_cal().write(false);
        }
        // 通过：每 500ms 翻转 LED；失败：每 100ms 翻转 LED。
        bsp::hardware::led.toggle();
        vTaskDelay(pdMS_TO_TICKS(drv8301_test_result.passed ? 500 : 100));
    }

    // flash.write_value("value1", 10);
    // flash.write_value("value2", 20);
    // flash.fresh();
    //  BSP::get_time_us();
    //  // bool ret = motor.init();
    //  bool ret=false;
    //  if (!ret) {
    //      while (1) {


    //         vTaskDelay(pdMS_TO_TICKS(1000));
    //     }
    // }

    // BSP::tim1_svpwm().register_timer_update_cb([]() {
    //     motor.fast_loop();
    // });

    // xTaskCreate(motor_slow_loop, "motor_slow_loop", 256, NULL, 1, &motor_slow_loop_task_handle);

    // while (1) {
    //     motor.handle_event(); // 阻塞等待并处理电机事件
    // }
}

// 启动阶段调用一次，构建的组件在整个应用生命周期内供 motor 使用。
void load_components_from_config(Encoder*& encoder, MotorDriver*& driver) {
    int encoder_type = 0;
    flash.read_value("encoder", encoder_type);
    switch (encoder_type) {
    case 0: encoder = new AS5600(flash, BSP::sensor_i2c_bus()); break;
    case 1: encoder = new AS5047P(flash); break;
    default: encoder = nullptr; break;
    }

    int driver_type = 0; // 未配置时沿用 DRV8301。
    flash.read_value("driver", driver_type);
    switch (driver_type) {
    case 0:
        driver = new DRV8301(
            flash, BSP::ic_spi_bus(), BSP::tim1_svpwm(), BSP::adc_measure(), BSP::drv8301_enable_gate(), BSP::drv8301_fault(),
            BSP::drv8301_dc_cal());
        break;
    default: driver = nullptr; break;
    }
}

void motor_slow_loop(void* param) {
    while (1) {
        motor.slow_loop();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
