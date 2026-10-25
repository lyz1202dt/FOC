# Agent 项目备忘

## 1. 工程约定

- 目标：STM32G431CBTx（Cortex-M4F）；HAL + FreeRTOS（CMSIS-RTOS v1）+ CMSIS-DSP；C++17/C11 混合。实际配置以 CMake、构建文件及 `FreeRTOSConfig.h` 为准。
- `Task/`：应用装配与板测；`BSP/`：外设封装及硬件映射；`LIB/`：电机、驱动、编码器、控制器、估计器、通信与工具库。
- `Core/`、`Drivers/`、`Middlewares/` 为生成代码或依赖；修改前须询问用户确认。
- 命名遵循 Google 风格，格式遵循 `.clang-format`；已有接口以源码为准，禁止无关重命名。
- 禁用 RTTI、异常及线程安全局部静态初始化；不得依赖这些能力。共享对象须明确初始化顺序。

## 2. 构建

- 构建依赖：CMake ≥ 3.22、Ninja、`arm-none-eabi-` GCC 工具链（加入 `PATH`）、Python3 及 PyYAML；配置阶段会检查 `import yaml`。预设使用 `cmake/gcc-arm-none-eabi.cmake`，Debug 为 `-Og -g3`，Release 为 `-Os -g0`。
- 在仓库根目录先配置、再构建；将以下命令中的 `Debug` 同时替换为 `Release` 可构建发布版本：

  ```bash
  cmake --preset Debug
  cmake --build --preset Debug
  ```

- 默认构建 `bootloader`、`foc` 和 `default_config`；可通过 `cmake --build --preset Debug --target foc`（或 `bootloader`、`default_config`）选择目标。`foc` 依赖 `default_config`，构建应用时也会生成默认配置镜像。
- 产物位于 `build/Debug/` 或 `build/Release/`：`bootloader.{elf,bin,hex,map}`、`foc.{elf,bin,hex,map}` 以及 `default_config.{bin,hex,json}`；默认配置由 `scripts/generate_flash_config.py` 根据 `default_config.yaml` 生成，YAML 仅在主机端解析。构建目录同时生成 `compile_commands.json`。
- 固件分区由 `cmake/firmware_layout.cmake` 统一定义：Bootloader 从 `0x08000000` 起占 24 KiB，应用从 `0x08006000` 起占 100 KiB，末尾 `0x0801F000` 起的 4 KiB 保留给配置。两个固件使用独立链接脚本；修改布局须同步核对链接脚本、应用向量表偏移和配置存储地址。
- CMake 仅构建，不包含烧录目标；烧录由独立的 `scripts/flash.py` 执行且不会自动构建，使用方式见 `Bootloader/README.md`。
- 顶层 CMake 递归收集 `BSP/`、`LIB/` 下的 `.cpp` / `.hpp`；`Task/app_main.cpp`、`Task/test.cpp` 显式列入源文件。新增 `Task/` 源文件须更新 `CMakeLists.txt`。
- `Task/test.hpp` / `Task/test.cpp` 提供 AS5047P、AS5600、DRV8301 板测；

## 3. 实时与并发约束

- `Motor::fast_loop()`、ISR、外设完成回调必须短小、有界、非阻塞；禁止堆分配、格式化输出、阻塞总线操作、Flash 操作及长临界区。
- ISR/线程共享数据须明确同步与所有权；`volatile` 不提供线程同步保证。临界区须恢复原中断屏蔽状态，不得无条件开中断。
- 总线 `init()` 必须在任务上下文、首次传输前执行；Flash 操作仅限任务上下文。
- `Motor::handle_event()` ，只能是函数最开始死等消息，消息一定接收成功，成功后下面开始分支判断。如果需要触发执行，在slow_loop中各自状态机中手动触发。

## 4. 电机执行流

| 入口 | 上下文 | 职责 |
|---|---|---|
| `Motor::fast_loop()` | TIM1 更新 ISR，经 `BSP::SVPWM` 回调进入 | 读取缓存测量值、获取编码器角度并发起异步采样、更新估计器、计算控制输出、切换控制器/估计器 |
| `Motor::slow_loop()` | 独立任务，每轮延时 50 ms | 状态反馈；`IDEL` 状态发送 `0x5A` 心跳 |
| `Motor::handle_event()` | 默认任务 | 阻塞等待事件队列，处理使能、模式切换、参考值、配置读写/保存 |

## 5. 外设与采样契约

外设类型位于 `bsp` 命名空间，实例位于 `bsp::hardware`；应用可通过 `BSP` 别名及 `BSP::sensor_i2c_bus()` 等接口访问。SPI/I2C/UART/FDCAN 均沿用现有队列与 HAL 回调分发机制。

| 外设 | 映射/用途 |
|---|---|
| SPI1 | DRV8301，片选 `ic_spi_cs` |
| SPI3 | AS5047P，片选 `sensor_spi_cs` |
| I2C1 | AS5600 |
| USART3 | `UartComm` |
| FDCAN1 | `BSP::fdcan1_bus()` |
| TIM1 | SVPWM 与 ADC 触发；相 A/B/C 对应 CH3/CH2/CH1，含互补输出 |
| ADC1 注入组 | Rank 1/2/3 = B 相电流、C 相电流、母线电压；由 TIM1 TRGO 触发 |
| TIM7 | `BSP::time_init()` / `BSP::get_time_us()`，微秒时间服务 |

- `LIB/tools/svpwm.*` 将相电压转换为比较值；`enable_timer()` 控制采样时基及更新中断，`enable_output()` 控制 PWM 功率输出。
- ADC 完成回调仅缓存原始值与时间戳至 `tools::Measure`（别名 `BSP::Measure`）。DRV8301 从缓存换算 B/C 相电流和母线电压，并按 `Ia = -(Ib + Ic)` 重建 A 相电流。
- 编码器 `get_value()` 返回缓存角度，并按需发起异步请求；完成回调更新角度及时间戳，使用 `value_is_available()` 判断有效性。AS5600 总线恢复由独立任务执行。

## 6. 核心模块

- `LIB/motor/`：`Motor` 管理控制环、事件和组件切换；`MotorDriver` 为驱动接口，`DRV8301` 实现寄存器通信、测量换算、校准及 PWM 输出控制。
- `LIB/controller/`：`Controller` 接口；`IDEL` 零电压、`VOC` 电压开环测试、`FOC` 电流控制器。
- `LIB/estimater/estimater.hpp`：估计器接口及默认角度透传实现。
- `LIB/encoder/`：编码器基类、AS5600、AS5047P 及 ABI/Hall 实现。
- `LIB/tools/`：Flash 键值存储、固定块内存池 `MemoryPoll`、环形缓冲区、测量缓存、PID、SVPWM。
