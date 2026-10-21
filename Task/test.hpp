#pragma once

// 1：编译该测试及其结果变量；0：完全排除。也可通过编译选项 -D 覆盖。
#ifndef TEST_AS5047P_ENABLED
#define TEST_AS5047P_ENABLED 0
#endif
#ifndef TEST_AS5600_ENABLED
#define TEST_AS5600_ENABLED 0
#endif
#ifndef TEST_DRV8301_ENABLED
#define TEST_DRV8301_ENABLED 0
#endif

// 在调度器启动后的任务上下文中选择一个调用，不可与电机控制环同时运行。
// 各测试自行初始化时间服务、Flash 和所需外设；包含无限延时循环，永不返回。
// 在 app_main.cpp 中包含 test.hpp 后调用，例如：as5047p_test();
#if TEST_AS5047P_ENABLED
[[noreturn]] void as5047p_test();
#endif
#if TEST_AS5600_ENABLED
[[noreturn]] void as5600_test();
#endif
#if TEST_DRV8301_ENABLED
[[noreturn]] void drv8301_test();
#endif
