#pragma once

#include "FreeRTOS.h"
#include "semphr.h"
#include <cstddef>
#include <cstdint>
#include <type_traits>

class Flash {
public:
    Flash();
    ~Flash();

    Flash(const Flash &) = delete;
    Flash &operator=(const Flash &) = delete;

    void init();

    void fresh();

    void remove(const char *key);

    template <typename T>
    bool write_value(const char *key, const T &value) {
        static_assert(std::is_trivially_copyable<T>::value,
                      "Flash only supports trivially copyable value types");
        return write_raw(key, &value, sizeof(T));
    }

    template <typename T>
    bool read_value(const char *key, T &value) {
        static_assert(std::is_trivially_copyable<T>::value,
                      "Flash only supports trivially copyable value types");
        return read_raw(key, &value, sizeof(T));
    }

    bool write_raw(const char *key, const void *value, size_t length);
    bool read_raw(const char *key, void *value, size_t length);

    static SemaphoreHandle_t write_mutex;
    static SemaphoreHandle_t read_mutex;

private:
    struct Data {
        char *key;
        void *p_data;
        int size;
        uint16_t key_length;
        Data *next;
    };

    struct StoragePage {
        uintptr_t address;
        uint32_t bank;
        uint32_t page;
    };

    struct PageHeader {
        uint32_t magic;
        uint16_t version;
        uint16_t header_size;
        uint32_t sequence;
        uint32_t blob_size;
        uint32_t blob_checksum;
        uint32_t reserved0;
        uint32_t reserved1;
        uint32_t reserved2;
    };

    struct PageFooter {
        uint32_t magic;
        uint32_t sequence;
    };

    struct BlobHeader {
        uint32_t magic;
        uint16_t version;
        uint16_t header_size;
        uint16_t used;
        uint16_t entry_count;
        uint32_t reserved;
    };

    struct EntryHeader {
        uint16_t key_length;
        uint16_t value_length;
        uint32_t reserved;
    };

    struct PageInfo {
        uint32_t sequence;
        size_t blob_size;
    };

    static constexpr size_t kStoragePageCount = 2U;
    static constexpr size_t kPageHeaderSize = 32U;
    static constexpr size_t kPageFooterSize = 8U;
    static constexpr uint32_t kPageMagic = 0x50464B56U;
    static constexpr uint32_t kFooterMagic = 0x54494D43U;
    static constexpr uint32_t kBlobMagic = 0x3142564BU;
    static constexpr uint16_t kBlobVersion = 1U;

    static_assert(sizeof(PageHeader) == kPageHeaderSize, "PageHeader must stay flash-word aligned");
    static_assert(sizeof(PageFooter) == kPageFooterSize, "PageFooter must stay flash-word aligned");
    static_assert(sizeof(BlobHeader) == 16U, "Unexpected BlobHeader layout");
    static_assert(sizeof(EntryHeader) == 8U, "Unexpected EntryHeader layout");

    static size_t storage_page_size();
    static size_t blob_capacity();
    static size_t align8(size_t value);
    static uint32_t checksum32(const uint8_t *data, size_t length);
    static void flash_clear_errors();
    static bool get_storage_page(size_t index, StoragePage &storage_page);
    static bool erase_storage_page(size_t index);
    static bool program_region(uintptr_t address, const uint8_t *data, size_t length);
    static bool program_page(size_t index, const uint8_t *blob, size_t blob_size, uint32_t sequence);
    static bool read_page(size_t index, PageInfo *info, uint8_t *blob, size_t capacity);
    static bool choose_active_page(size_t &active_page, PageInfo &active_info);
    static bool read_blob_header(const uint8_t *blob, size_t length, BlobHeader &header);
    static bool read_entry_header(const uint8_t *blob, size_t used, size_t offset, EntryHeader &entry);
    static bool entry_total_size(const EntryHeader &entry, size_t *total_size);
    static bool validate_blob(const uint8_t *blob, size_t length, size_t *used);
    static bool ensure_mutex(SemaphoreHandle_t &mutex);
    static bool take_mutex(SemaphoreHandle_t &mutex);
    static void give_mutex(SemaphoreHandle_t mutex);
    static bool get_key_length(const char *key, size_t *length);
    static char *duplicate_key(const char *key, size_t length);

    void clear_ram();

    Data *data_head_{nullptr};
    bool initialized_{false};
    bool dirty_{false};
};
