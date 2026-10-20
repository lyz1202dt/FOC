# Agent 项目备忘

## 1. 工程约定

- 目标：STM32G431CBTx（Cortex-M4F）；HAL + FreeRTOS（CMSIS-RTOS v1）+ CMSIS-DSP；C++17/C11 混合。实际配置以 CMake、构建文件及 `FreeRTOSConfig.h` 为准。
- `Task/`：应用装配与板测；`BSP/`：外设封装及硬件映射；`LIB/`：电机、驱动、编码器、控制器、估计器、通信与工具库。
- `Core/`、`Drivers/`、`Middlewares/` 为生成代码或依赖；修改前须询问用户确认。
- 命名遵循 Google 风格，格式遵循 `.clang-format`；已有接口以源码为准，禁止无关重命名。
- 禁用 RTTI、异常及线程安全局部静态初始化；不得依赖这些能力。共享对象须明确初始化顺序。

## 2. 实时与并发约束

- `Motor::fast_loop()`、ISR、外设完成回调必须短小、有界、非阻塞；禁止堆分配、格式化输出、阻塞总线操作、Flash 操作及长临界区。
- ISR/线程共享数据须明确同步与所有权；`volatile` 不提供线程同步保证。临界区须恢复原中断屏蔽状态，不得无条件开中断。
- 总线 `init()` 必须在任务上下文、首次传输前执行；Flash 操作仅限任务上下文。

## 3. 启动与电机执行流

调度器启动后，`Core/Src/app_freertos.c::StartDefaultTask()` 调用 C ABI 入口 `Task/app_main.cpp::app_main()`。后者持有 `Flash`、容量为 2 的 `CommManager`、`UartComm` 和 `Motor`。

初始化顺序：
1. 初始化 TIM7 时间服务、SPI/I2C 总线、Flash。
2. 按配置构造编码器与驱动器，通过 `Motor::install_driver()` 装配；**仅允许在首次 `Motor::init()` 前调用**。
3. 初始化通信及电机。`Motor::init()` 创建事件队列、估计器、控制器，初始化编码器与驱动器，注册 TIM1 更新回调，启动 ADC/定时器并执行电流校准。
4. 初始化完成后保持功率输出关闭，等待显式使能。

| 入口 | 上下文 | 职责 |
|---|---|---|
| `Motor::fast_loop()` | TIM1 更新 ISR，经 `BSP::SVPWM` 回调进入 | 读取缓存测量值、获取编码器角度并发起异步采样、更新估计器、计算控制输出、切换控制器/估计器 |
| `Motor::slow_loop()` | 独立任务，每轮延时 50 ms | 状态反馈；`IDEL` 状态发送 `0x5A` 心跳 |
| `Motor::handle_event()` | 默认任务 | 阻塞等待事件队列，处理使能、模式切换、参考值、配置读写/保存 |

## 4. 外设与采样契约

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

## 5. 核心模块

- `LIB/motor/`：`Motor` 管理控制环、事件和组件切换；`MotorDriver` 为驱动接口，`DRV8301` 实现寄存器通信、测量换算、校准及 PWM 输出控制。
- `LIB/controller/`：`Controller` 接口；`IDEL` 零电压、`VOC` 电压开环测试、`FOC` 电流控制器。
- `LIB/estimater/estimater.hpp`：估计器接口及默认角度透传实现。
- `LIB/encoder/`：编码器基类、AS5600、AS5047P 及 ABI/Hall 实现。
- `LIB/tools/`：Flash 键值存储、固定块内存池 `MemoryPoll`、环形缓冲区、测量缓存、PID、SVPWM。

## 7. Flash 与链接布局

- `Flash::init()` 在调度器启动后的任务中加载配置；`write_value()` / `write_raw()` 修改 RAM 副本，`fresh()` 持久化，`clear()` 擦除两页。禁止在 ISR 中调用。
- `STM32G431XX_FLASH.ld`：程序占用 124 KiB Flash、32 KiB RAM；末尾 4 KiB（两页）用于键值存储。存储通过序号、校验及提交标记选择有效页。
- 修改 Flash 布局时，必须同步核对链接脚本及 `Flash` 页地址计算。

## 8. 构建与板测

- 构建：`cmake --preset Debug` 或 `Release`，再执行 `cmake --build --preset Debug` 或对应配置的构建命令。
- 顶层 CMake 递归收集 `BSP/`、`LIB/` 下的 `.cpp` / `.hpp`；`Task/app_main.cpp`、`Task/test.cpp` 显式列入源文件。新增 `Task/` 源文件须更新 `CMakeLists.txt`。
- `Task/test.hpp` / `Task/test.cpp` 提供 AS5047P、AS5600、DRV8301 板测；通过 `TEST_AS5047P_ENABLED`、`TEST_AS5600_ENABLED`、`TEST_DRV8301_ENABLED` 控制编译。

