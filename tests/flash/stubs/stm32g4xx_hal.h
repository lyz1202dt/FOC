#pragma once
#include <cstddef>
#include <cstdint>
constexpr uintptr_t FLASH_BASE = 0x08000000U;
constexpr size_t FLASH_SIZE = 128U * 1024U;
constexpr size_t FLASH_PAGE_SIZE = 2048U;
constexpr uint32_t FLASH_BANK_1 = 1U;
constexpr uint32_t FLASH_TYPEERASE_PAGES = 0U;
constexpr uint32_t FLASH_TYPEPROGRAM_DOUBLEWORD = 0U;
constexpr uint32_t FLASH_FLAG_EOP = 1U;
constexpr uint32_t FLASH_FLAG_ALL_ERRORS = 0xFEU;
#define __HAL_FLASH_CLEAR_FLAG(flags) ((void)(flags))
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
struct FLASH_EraseInitTypeDef {
    uint32_t TypeErase;
    uint32_t Banks;
    uint32_t Page;
    uint32_t NbPages;
};
HAL_StatusTypeDef HAL_FLASH_Unlock();
HAL_StatusTypeDef HAL_FLASH_Lock();
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t word);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef* erase, uint32_t* page_error);
