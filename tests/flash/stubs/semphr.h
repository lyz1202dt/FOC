#pragma once
#include "FreeRTOS.h"
struct FakeMutex { bool held; };
using SemaphoreHandle_t = FakeMutex*;
SemaphoreHandle_t xSemaphoreCreateMutex();
BaseType_t xSemaphoreTake(SemaphoreHandle_t mutex, uint32_t timeout);
BaseType_t xSemaphoreGive(SemaphoreHandle_t mutex);
