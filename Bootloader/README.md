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

## 独立烧录脚本

CMake 只构建产物，不包含烧录目标。先构建，再通过独立脚本调用 J-Link：

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
`--exe` 指定 J-Link Commander 路径（默认 `JLinkExe`），多探针时使用
`--serial-no` 指定序列号。默认芯片为 `STM32G431CB`，接口为 SWD，速度为
4000 kHz，可分别通过 `--device`、`--interface`、`--speed` 覆盖。

脚本不会自动构建；实际烧录前检查所选产物是否存在。仅选择 `config` 或
`all` 时擦除配置区并写入、校验默认配置；其余选择保留配置区。
不执行整片擦除，完成后复位运行。只烧录应用时，板上须已有匹配布局的
Bootloader；只烧录 Bootloader 时，运行依赖板上已有的应用。
VS Code 的 Flash 任务先构建 Debug，再烧录 Bootloader 和应用，保留配置。
