# 智能体项目备忘

本仓库是 STM32G431 FOC 固件：STM32CubeMX 生成 C 代码 + FreeRTOS + 用户侧 C++17。

## 入口链路

- `Core/Src/main.c` 完成 HAL/外设初始化，随后 `MX_FREERTOS_Init()` 创建默认任务。
- `Core/Src/app_freertos.c::StartDefaultTask()` 调用 C ABI 入口 `app_main(NULL)`。
- `Task/app_main.cpp` 负责装配全局对象并初始化 BSP、通信、编码器、驱动器和电机。
- `TIM1` 更新中断走 `HAL_TIM_PeriodElapsedCallback()` -> `BSP::SVPWM::timer_irq()` -> `Motor::fast_loop()`。
- `ADC1` 注入转换由 `TIM1 TRGO` 触发，结果在 `BSP::Measure` 缓存；`TIM7` 提供 us 级时间戳。

## 分层现状

- `Core/`、`Drivers/`、`Middlewares/`：CubeMX/HAL/CMSIS/FreeRTOS 代码，除非必须集成外设，否则少改。
- `BSP/`：HAL 包装、板级单例和 HAL 回调分发；包含 GPIO、SPI/I2C 队列、ADC 采样、SVPWM。
- `Task/`：应用装配层。当前创建 `Flash`、`AS5600`、`DRV8301`、`Estimater`、`FOC`、`Motor`、`PackParser`。
- `LIB/motor/`：电机编排与 `MotorDriver` 抽象；`DRV8301` 负责栅极驱动、相电流/母线电压读取和 SVPWM 输出。
- `LIB/encoder/`：`Encoder` 基类和具体编码器；`AS5600` 通过构造注入 I2C 总线并异步连续采样。
- `LIB/controller/`：控制器接口和 `FOC`，但当前 `FOC` 仍是占位实现，输出三相 0V。
- `LIB/estimater/`：估计器接口，当前基本透传编码器角度；无感模式只是预留路径。
- `LIB/comm/`：UART 帧收发和命令分发；`UartComm` 直接挂 `UART_HandleTypeDef`，RX 使用 DMA circular + IDLE/HT/TC 事件推进环形缓冲，TX 使用 DMA 完成中断；已注册 `0x01` 电机使能、`0x02` 目标值更新。
- `LIB/tools/flash.*`：两页轮换的 Flash KV 存储，使用 FreeRTOS mutex；不得在快环里调用。

## 实时约束

`Motor::fast_loop()` 在 TIM1 IRQ 中运行，保持短小、有界、无阻塞。该路径及其下游避免：

- 堆分配、`std::function`、`unordered_map`、`std::string`、格式化输出。
- 阻塞 HAL 调用、SPI/I2C 事务、Flash 擦写、`vTaskDelay()`、长临界区。
- 只读取已缓存的 ADC/编码器数据，运行控制计算，然后更新 PWM。

注意：`Motor` 里已有 `std::function` 快环结束回调，通信解析也使用 `std::function/unordered_map`；后续不要把这些机制继续扩进实时中断路径。

## 设计注意

- 总体分层方向合理：CubeMX 与用户 C++ 隔离，BSP 屏蔽 HAL，`Task` 做装配，`Motor` 依赖 `Encoder/MotorDriver/Controller/Estimater` 接口。
- 主要待收口点：`Motor` 内部指针目前是 public，`Task` 直接访问 `motor.controller_`；新代码优先加窄接口而不是扩大 public 状态。
- 具体硬件对象优先放在 `Task/app_main.cpp` 或 `BSP::hardware.cpp`；新增传感器/驱动器尽量用构造注入，减少直接抓 BSP 单例。
- `PackParser` 回调可能从 `UartComm` 的 UART Rx IRQ 回调链路执行，回调逻辑要轻量；复杂处理应转到任务上下文。
- 保留 `app_main` 的 C ABI 边界，避免无关 CubeMX/供应商代码改动。

## 构建检查

使用 preset 构建，输出在 `build/Debug`：

```sh
git diff --check
cmake --build --preset Debug
```
