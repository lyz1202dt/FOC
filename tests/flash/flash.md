# Flash 配置日志（格式 v2）

`write_value()` / `write_raw()` 和 `remove()` 只修改 RAM 链表。
`read_value()` / `read_raw()` 从 RAM 读取，要求保存长度与请求长度相等。
所有接口只用于任务上下文；在调度器启动后、共享对象前调用 `init()`。

## 保存流程

`fresh()` 每次扫描已提交的 Flash 日志，按 key 比较 RAM 当前值的长度和内容，
只追加新增、修改和删除记录。不维护每项配置的 dirty 标志或已保存值副本。
相同值、修改后改回原值、未保存的新条目被删除，都不会引起编程或擦除。

同一次 `fresh()` 的所有变化组成一个批次：先写批次头和记录，回读确认后，
最后写独立的提交双字。批次提交成功后，才对被替代的旧记录写失效标记。
删除也追加日志记录，避免重启后旧值重新出现。

`fresh()` 返回 `bool`。提交前失败返回 `false`，RAM 修改保留，调用方可以重试。
提交成功后，旧记录失效标记写入失败不撤销提交，返回 `true`，下一次实际变更将整理换页。
启动恢复始终以同 key 最后的已提交记录为准，不依赖旧记录是否成功写入失效标记。

## 物理布局

STM32G431CB 的最后两页各为 2048 字节，地址仍为 `0x0801F000` 和 `0x0801F800`。
链接脚本继续为程序分配前 124 KiB，不修改保留区。

每页布局如下，整数按目标芯片的小端内存布局保存：

| 结构 | 大小 | 字段 |
| --- | ---: | --- |
| PageHeader | 16 | magic(u32)、version(u16)、header_size(u16)、sequence(u32)、checksum(u32) |
| PageActivation | 8 | magic(u32)、sequence(u32)，最后激活新页 |
| BatchHeader | 16 | magic(u32)、total_size(u16)、entry_count(u16)、checksum(u32)、reserved(u32) |
| EntryHeader | 8 | key_length(u16)、value_length(u16)、type(u16)、total_size(u16) |
| 条目内容 | 变长 | key（不含终止符）+ value，整体补齐到 8 字节边界 |
| ObsoleteMarker | 8 | 独立的失效标记双字，每条记录各有一个 |
| BatchCommit | 8 | magic(u32) 及其按位取反值(u32)，位于本批所有条目之后 |

后续批次紧接上一批提交记录追加。PageHeader 的 `header_size` 为 24，包含激活标记。
条目类型 1 为设置，类型 2 为删除；删除记录的 value_length 为 0。
每条记录占 `16 + align8(key_length + value_length)` 字节，每批固定开销 24 字节。

页头校验覆盖其前 12 字节。批次使用 32 位 FNV-1a 校验，依次覆盖批次头前 8 字节
及所有记录的头、内容和对齐填充，不包含可变的失效标记区。
reserved 和填充字节为 `0xFF`。

失效标记区最初保持物理擦除态，不能预先编程全 `0xFF`；退休旧记录时才编程一次。
失效标记为 magic `0x32444C4F` 及其按位取反值。所有标记均独占 64 位编程单元。
通用编程函数跳过全 `0xFF` 双字，防止无意义地消耗 ECC 位。

## 页满整理与恢复

空间足够时只追加，不擦页。批次放不下或尾部曾发生写入失败时，才将 RAM 中全部
有效配置写入另一页。目标页先擦除，完整快照提交并校验后，最后写页激活标记。
旧页保留到下一次整理需要它时再擦除，每次整理只擦除一个目标页。
目标页即使读出全 `0xFF` 也先擦除，避免复用掉电时可能已消耗 ECC 位的双字。

`write_raw()` 按整理后的快照检查容量，所有有效配置必须能放进单页：

```text
24（页头及激活）+ 24（批次头及提交）
+ Σ[16 + align8(key_length + value_length)] <= 2048
```

`init()` 选择代次较新的已激活页，序号比较支持 u32 回绕。只恢复完整提交的批次。
不完整尾部保留此前的已提交配置，下次保存实际变化时整理换页，不覆盖残留双字。
不兼容格式、无可用页但存在非空内容、或已提交批次的结构/校验错误，会调用 `clear()`
重置两页；不迁移旧格式。内存分配失败不会触发擦除。

`clear()` 仍为直接擦除两页，成功后清空 RAM；不保证擦除过程中的掉电原子性。

## 验证

在 Linux 主机运行：

```sh
./tests/flash/run.sh
```

测试直接编译生产 `flash.cpp`，模拟两页、互斥锁、64 位一次编程限制和页擦除，
并启用 UBSan。覆盖内容比较、删除、容量边界、换页、格式重置、随机操作，
以及每次编程/擦除前、半写入、完整写入后和不可见 ECC 消耗的故障注入。

模拟不覆盖真实芯片的不可纠正 ECC / NMI 行为，也不能替代实际板子的断电测试。

## 构建时生成默认配置与同步下载

主机需要 Python 3 和 PyYAML（`python3 -m pip install PyYAML`）。

```sh
cmake --preset Debug
cmake --build --preset Debug
# 同一次 J-Link 会话下载程序并强制覆盖全部配置，完成后复位运行
cmake --build --preset Debug --target flash
```

Release 使用相应 preset。构建本身不会连接硬件。
每个构建目录生成 `default_config.bin`（完整 4096 字节镜像）、
`default_config.hex`（稀疏的已使用双字）和 `default_config.json`（配置清单）。
YAML 或生成脚本发生变化会自动重新生成，不依赖固件重新链接。

`default_config.yaml` 的 `config` 是容器，不进入 key；其下路径使用 `/` 连接。
叶子必须为 `[类型, 值]`，仅支持 int32/uint32/int64/uint64/float32/float64。
整数和浮点数按小端序编码；不支持字符串值，`str` 类型会使构建失败。
key 使用 ASCII，不含终止符。
数值类型必须匹配固件读取变量，尤其 `foc/offset` 必须是 float32。
重复 YAML key、非法类型、越界值、非有限浮点值和超出单页容量均使构建失败。

镜像第一页 sequence=1，包含一个完整提交批次；第二页全为擦除态。
`flash` 目标先下载程序，再显式擦除 `0x0801F000..0x0801FFFF` 两页，
下载稀疏 HEX，使用完整 BIN 回读校验两页，然后复位运行。
显式擦除第二页可防止旧页较大的 sequence 覆盖默认配置的选择结果。
HEX 不包含全 FF 双字，避免预先消耗空闲区和失效标记的 ECC 编程机会。
下载期间不要断电；该流程不保证程序与配置整体更新的掉电原子性。

可在 configure 时指定 `-DJLINK_EXECUTABLE=/path/to/JLinkExe`、
`-DJLINK_SERIAL=探针序列号`；连接多个探针时必须指定序列号。
目标固定为 STM32G431CB、SWD、4000 kHz。

VS Code 的 `Flash` task 使用同一 CMake 目标；J-Link 调试配置将其作为
preLaunchTask，随后仅复位/暂停，不再执行第二次程序下载。
ELF 仍用于调试符号。直接用其他工具只下载 `foc.elf` 不会更新默认配置。

`./tests/flash/run.sh` 还会生成包含全部支持类型的测试镜像，使用生产
`Flash::init()` 加载，并验证追加保存和再次加载；同时检查输入校验和稀疏 HEX。
真实 J-Link 擦除、校验及后续保存行为仍需板上验证。
