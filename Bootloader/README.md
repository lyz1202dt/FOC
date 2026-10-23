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
两个链接脚本通过 `PHDRS` 显式划分 Flash、RAM 初始化数据和 RAM 零初始化段，
排除 ELF 文件头及分区前的对齐填充。应用 ELF 的首个 `LOAD` 段从
`0x08006000` 开始，避免 OpenOCD 按段烧录时覆盖 Bootloader。
可使用 `arm-none-eabi-readelf -l build/Debug/foc.elf` 核对加载段地址。

## 启动与烧录

Bootloader 不初始化 HAL、串口、PLL 或 RTOS，直接读取 APP 向量表中的 MSP
和复位入口，无条件跳转。跳转前关闭 SysTick、清理 NVIC 和待处理系统中断、
设置 VTOR；最后通过汇编切换 MSP 并恢复中断使能。
没有接收固件、镜像校验、有效标记或恢复模式，启动前必须烧入匹配布局的 APP。

`Bootloader/CMakeLists.txt` 显式列出参与编译的源码。此前复制的 F1 系统/MSP
文件、HAL 配置、中断及 syscall 文件均未参与当前最小 Bootloader 构建。

## 独立烧录脚本

CMake 只构建产物，不包含烧录目标。先构建，再通过独立脚本调用 OpenOCD：

```bash
cmake --preset Release
cmake --build --preset Release

# 仅更新应用，保留 Bootloader 和已有配置
python3 scripts/flash.py --preset Release --image app

# 只更新 Bootloader
python3 scripts/flash.py --preset Release --image bootloader

# 一次连接烧录两个固件，保留已有配置
python3 scripts/flash.py --preset Release --image bootloader app

# 仅恢复默认配置，或完整初始化（包含默认配置）
python3 scripts/flash.py --preset Release --image config
python3 scripts/flash.py --preset Release --image all

# 预览命令，不连接探针，允许产物尚未生成
python3 scripts/flash.py --preset Release --image all --dry-run
```

`--preset` 默认为 `Debug`，也可通过 `--build-dir` 指定产物目录。
`--exe` 指定 OpenOCD 路径（默认 `openocd`），`--config` 指定项目配置
（默认仓库根目录 `openocd.cfg`），`--search` 可重复添加脚本搜索目录。
不预设探针类型；使用前须在 `openocd.cfg` 中设置 `PROBE_INTERFACE`，或通过
`--interface` 显式传入 OpenOCD 适配器配置，也可通过 `--config` 提供完整配置。
默认使用 SWD + STM32G431，速度为 4000 kHz；`--speed` 可覆盖速率，
`--serial-no` 选择探针。更换探针只需选用 OpenOCD 的适配器配置，无需修改 Python：

```bash
# CMSIS-DAP
python3 scripts/flash.py --image app --interface interface/cmsis-dap.cfg
# ST-Link（HLA 驱动使用 hla_swd；具体名称以本机 OpenOCD 配置为准）
python3 scripts/flash.py --image app --interface interface/stlink.cfg --transport hla_swd
```

下载和调试共用 `openocd.cfg`。要长期更换探针，可修改其中的
`PROBE_INTERFACE` / `PROBE_TRANSPORT` 设置；例如在文件开头添加
`set PROBE_INTERFACE interface/cmsis-dap.cfg`。VS Code 调试及 Flash 任务也使用此设置。
目标芯片配置为 `target/stm32g4x.cfg`。

脚本不会自动构建；实际烧录前检查所选产物是否存在。仅选择 `config` 或
`all` 时擦除配置区并写入、校验默认配置；其余选择保留配置区。
不执行整片擦除，完成后复位运行。只烧录应用时，板上须已有匹配布局的
Bootloader；只烧录 Bootloader 时，运行依赖板上已有的应用。
VS Code 的 Flash 任务先构建 Debug，再烧录 Bootloader 和应用，保留配置。

## OpenOCD 调试

安装 OpenOCD（需支持 STM32G4）、ARM GDB（当前 VS Code 配置使用
`/usr/bin/gdb-multiarch`）和 VS Code Cortex-Debug 扩展。
选择 `OpenOCD Debug (FOC)` 或 `OpenOCD Debug (Bootloader)` 启动调试；
前置 Flash 任务构建并下载两个 Debug 固件，保留已有配置，然后启动 OpenOCD。
调试器执行 `reset halt` 并运行到所选 ELF 的 `main`，应用通过 Bootloader 跳转进入。
`preLaunchTask: Flash` 必须保持启用：当前 `overrideLaunchCommands` 只复位，
不会下载固件。若跳过 Flash，板上固件可能与本地 ELF 不一致，导致入口断点无法命中。
启动时停在 `main`；若要停在更早的复位入口，可将 `runToEntryPoint` 改为
`Reset_Handler`（FOC 配置仍需先经过 Bootloader）。
已开启 `showDevDebugOutput: raw`，若仍不停住，可从“调试控制台”查看
复位、入口断点设置和运行结果。
首次使用若需要默认配置，先运行 `python3 scripts/flash.py --image all`。
可按本机工具安装位置调整 `gdbPath` / `objdumpPath`。

也可以在仓库根目录手动启动 GDB Server（默认 GDB 端口 3333）：

```bash
openocd -f openocd.cfg
# 在另一终端连接；先完成构建和下载
arm-none-eabi-gdb build/Debug/foc.elf
# GDB 内执行：
# target extended-remote localhost:3333
# monitor reset halt
# break main
# continue
```

下载脚本启动一次性 OpenOCD 进程，调试会话则持有探针；下载前先结束已有调试会话。
