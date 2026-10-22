# Bootloader + 单 APP

顶层 CMake 同时管理 `bootloader` 和 `foc` 两个独立目标。各自拥有启动文件、
系统初始化、链接脚本及 ELF/BIN/HEX/MAP 产物。分区参数位于
`cmake/firmware_layout.cmake`，由链接选项传入两个链接脚本。

| 区域 | 起始地址 | 大小 |
| --- | --- | --- |
| Bootloader | `0x08000000` | 24 KiB |
| APP (`foc`) | `0x08006000` | 100 KiB |
| 配置存储 | `0x0801F000` | 4 KiB |

APP 的 `SystemInit()` 通过目标编译定义设置向量表偏移，支持从 Bootloader
跳转或由调试器直接启动 APP。配置存储仍使用芯片末尾两页。

## 构建

在仓库根目录执行：

```bash
cmake --preset Release
cmake --build --preset Release

# 只构建一个固件
cmake --build --preset Release --target bootloader
cmake --build --preset Release --target foc
```

将 `Release` 替换为 `Debug` 可构建调试版本。产物均在 `build/<配置>/`：
`bootloader.{elf,bin,hex,map}`、`foc.{elf,bin,hex,map}`。
ELF/HEX 包含地址；烧录 BIN 时需显式指定上表对应的起始地址。

## 启动与烧录

Bootloader 不初始化 HAL、串口、PLL 或 RTOS，直接读取 APP 向量表中的 MSP
和复位入口，无条件跳转。跳转前关闭 SysTick、清理 NVIC 和待处理系统中断、
设置 VTOR；最后通过汇编切换 MSP 并恢复中断使能。
没有接收固件、镜像校验、有效标记或恢复模式，启动前必须烧入匹配布局的 APP。

`Bootloader/CMakeLists.txt` 显式列出参与编译的源码。此前复制的 F1 系统/MSP
文件、HAL 配置、中断及 syscall 文件均未参与当前最小 Bootloader 构建。

现有 J-Link 烧录入口：

```bash
cmake --build --preset Release --target flash
```

该目标先构建并烧录两个 ELF，然后沿用原行为擦除配置区并写入默认配置。
不执行整片擦除。仅构建固件不会连接探针或操作板卡。
