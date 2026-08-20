# 智能体项目说明

本项目是 STM32G431 FOC 固件，使用 STM32CubeMX 生成的 C 代码、
FreeRTOS，以及用户侧 C++17 模块。

## 程序入口

- 硬件初始化从 `Core/Src/main.c` 开始。
- FreeRTOS 在 `Core/Src/app_freertos.c` 中创建 `StartDefaultTask()`。
- `StartDefaultTask()` 调用 `app_main(NULL)`。
- 应用层入口位于 `Task/app_main.cpp`。
- `TIM1` 更新中断经由 `BSP::SVPWM::timer_irq()` 分发，并调用
  `Motor::fast_loop()`。

## 目录职责

- `Core/`、`Drivers/`、`Middlewares/`：STM32CubeMX、HAL、CMSIS 和
  FreeRTOS 生成/供应商代码。除非任务明确要求修改 HAL 或 CubeMX 集成，
  否则尽量不要改动。
- `BSP/`：板级支持层。负责包装 HAL 句柄，并把 HAL 回调分发到 C++ 对象。
  典型模块包括 GPIO、SPI、I2C、ADC 采样、SVPWM、板级单例。
- `LIB/motor/`：电机协调逻辑和栅极驱动抽象。
- `LIB/encoder/`：编码器基类和 AS5600 等具体编码器。
- `LIB/estimater/`：控制环状态估计接口，当前透传编码器角度，后续可替换为无感估计。
- `LIB/controller/`：控制算法抽象和 FOC 实现。
- `LIB/tools/`：工具模块，例如基于 Flash 的键值存储。
- `Task/`：应用装配层和运行入口。

## 当前架构

- `app_main.cpp` 是装配层，负责创建具体的 `Flash`、`AS5600`、`DRV8301`、
  `Estimater`、`FOC` 和 `Motor` 对象。
- `Motor` 通过引用接收 `Flash`、`Encoder`、`MotorDriver`、`Estimater` 和 `Controller`。
- `Motor::fast_loop()` 是由中断触发的快环路径，必须保持有界且确定。
- `BSP::hardware.cpp` 持有板级单例，并负责 HAL 回调转发。

## 实时性约束

TIM1 更新路径是快速控制环。在该路径及其下游应避免：

- 动态分配，例如 `new` 或会分配堆内存的容器
- 阻塞式 HAL 调用
- Flash 擦写
- 字符串格式化或 `std::string`
- `std::function` 或较重的类型擦除回调
- 过长的临界区
- I2C/SPI 事务，除非已经证明它们非阻塞且耗时有界

传感器和通信工作应尽量异步执行，或放在任务上下文中处理。快环只应读取
已经采样好的数据、运行控制计算，并更新 PWM 输出。

## 构建与验证

使用 CMake preset，不要直接使用普通的 `build/` 目录：

```sh
cmake --build --preset Debug
```

普通 `build/` 目录可能被配置成宿主机编译器，导致 ARM FreeRTOS 汇编报错。
Preset 构建会使用 `cmake/gcc-arm-none-eabi.cmake`，输出目录为 `build/Debug`。

常用检查：

```sh
git diff --check
cmake --build --preset Debug
```

## 编码约定

- 优先使用已有 BSP 包装，不要在 `LIB` 或 `Task` 中随意直接访问 HAL。
- 硬件相关的具体对象创建应放在 `Task/app_main.cpp` 或专门的板级装配层。
- 保留与生成 C 代码交互所需的 C ABI 边界，例如 `app_main`。
- 避免无关的 CubeMX 或供应商代码改动，除非避免这些改动会导致别的位置有大量代码补偿。
