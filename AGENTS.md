# 项目备忘


## 编码要求

- 除非一个很长的逻辑在多处被重复使用，否则不要将其写成一个函数/类成员函数，如果有必要添加一个只在类中被用到的函数，在将其作为私有成员函数添加。
- 按照谷歌风格的命名方式去命名新变量/函数

## 实时与内存约束

- Motor::fast_loop快环和接收中断保持短小、有界、无阻塞；不增加堆分配、格式化输出、阻塞总线事务、Flash 操作或长临界区。
- ISR/线程共享状态必须明确同步与所有权；不能仅依赖 `volatile`。中断保护须恢复原屏蔽状态，不能无条件开中断。
- `LIB/tools/memorypoll.*` 是固定块池。构造时一次堆分配，附带每块一位的分配状态；无堆分配、阻塞或链表扫描。特权线程与可屏蔽 IRQ 共享；不支持 NMI/HardFault、非特权线程或多核共享。

## 入口与职责

- STM32G431、CubeMX HAL/CMSIS、FreeRTOS，用户代码使用 C++17。`Core/`、`Drivers/`、`Middlewares/` 是生成代码或依赖，非必要不修改。
- `Core/Src/app_freertos.c::StartDefaultTask()` 调用 C ABI 入口 `Task/app_main.cpp::app_main()`。
- `Task/app_main.cpp` 装配 `Flash`、`CommManager`、`Motor`、`UartComm`，初始化外设和电机，注册 TIM1 更新回调，在线程中循环调用 `Motor::handle_event()`。
- `BSP/hardware.cpp` 提供总线、GPIO、TIM1 SVPWM、ADC 测量及 TIM7 时间服务。快环由 TIM1 更新回调进入 `Motor::fast_loop()`；ADC 注入转换回调缓存测量值。
- LIB 提供核心的通讯/编码器驱动/估计器/控制器/以及其他必要的算法和工具函数等等
- Motor分三部分同时执行
  fast\_loop();  在PWM更新中断中调用，执行FOC算法
  slow\_loop();   维护状态更新，LED指示，心跳保活包等
  handle\_event();    处理事件（在线程上下文执行）

## 构建

```sh
cmake --build --preset Debug
```

- 工具链为 `arm-none-eabi-*`，构建目录 `build/Debug`；CMake 自动收集 `BSP/`、`LIB/` 下的 C++ 源文件。

