#pragma once

#include "FreeRTOS.h"
#include "semphr.h"
#include <cstddef>
#include <cstdint>
#include <type_traits>

// Task context only. Initialize after scheduler startup, before sharing the object.
class Flash {
public:
    Flash();
    ~Flash();

    Flash(const Flash&) = delete;
    Flash& operator=(const Flash&) = delete;

    // Load the current journal. Unknown/incompatible formats are cleared, not migrated.
    void init();

    // Compare RAM with committed Flash values and append one atomic batch of changes.
    // False preserves RAM for retry. A committed batch remains durable even if retiring
    // an obsolete record fails; recovery always resolves duplicate keys by log order.
    bool fresh();

    // Erase both storage pages, then clear RAM on success. Not power-fail atomic.
    // On erase failure, return false and retain RAM; Flash may be partly erased.
    bool clear();

    void remove(const char* key);

    template <typename T>
    bool write_value(const char* key, const T& value) {
        static_assert(std::is_trivially_copyable<T>::value, "Flash only supports trivially copyable value types");
        return write_raw(key, &value, sizeof(T));
    }

    template <typename T>
    bool read_value(const char* key, T& value) {
        static_assert(std::is_trivially_copyable<T>::value, "Flash only supports trivially copyable value types");
        return read_raw(key, &value);
    }

    bool write_raw(const char* key, const void* value, size_t length);
    bool read_raw(const char* key, void* value);

    static SemaphoreHandle_t write_mutex;
    static SemaphoreHandle_t read_mutex;

private:
    struct Data {
        char* key;
        void* p_data;
        int size;
        uint16_t key_length;
        Data* next;
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
        uint32_t checksum;
    };

    struct PageActivation {
        uint32_t magic;
        uint32_t sequence;
    };

    struct BatchHeader {
        uint32_t magic;
        uint16_t total_size;
        uint16_t entry_count;
        uint32_t checksum;
        uint32_t reserved;
    };

    struct BatchCommit {
        uint32_t magic;
        uint32_t inverse;
    };

    struct EntryHeader {
        uint16_t key_length;
        uint16_t value_length;
        uint16_t type;
        uint16_t total_size;
    };

    // Every marker occupies its own doubleword, left physically unprogrammed until
    // retirement. It is excluded from batch checksums and is advisory on recovery.
    struct ObsoleteMarker {
        uint32_t magic;
        uint32_t inverse;
    };

    struct PageInfo {
        uint32_t sequence;
        size_t committed_end;
        bool appendable;
    };

    struct RecordView {
        EntryHeader header;
        const uint8_t* key;
        const uint8_t* value;
        uintptr_t obsolete_address;
    };

    struct RecordCursor {
        size_t offset;
        uint16_t remaining;
    };

    static constexpr size_t kStoragePageCount = 2U;
    static constexpr size_t kPagePrefixSize = sizeof(PageHeader) + sizeof(PageActivation);
    static constexpr size_t kBatchOverhead = sizeof(BatchHeader) + sizeof(BatchCommit);
    static constexpr uint32_t kPageMagic = 0x324C564BU;
    static constexpr uint16_t kFormatVersion = 2U;
    static constexpr uint32_t kActivationMagic = 0x32544341U;
    static constexpr uint32_t kBatchMagic = 0x3242544BU;
    static constexpr uint32_t kCommitMagic = 0x32544D43U;
    static constexpr uint32_t kObsoleteMagic = 0x32444C4FU;
    static constexpr uint16_t kSetEntry = 1U;
    static constexpr uint16_t kDeleteEntry = 2U;
    static constexpr uint32_t kHashSeed = 2166136261UL;

    static_assert(sizeof(PageHeader) == 16U, "Unexpected PageHeader layout");
    static_assert(sizeof(PageActivation) == 8U, "Unexpected PageActivation layout");
    static_assert(sizeof(BatchHeader) == 16U, "Unexpected BatchHeader layout");
    static_assert(sizeof(BatchCommit) == 8U, "Unexpected BatchCommit layout");
    static_assert(sizeof(EntryHeader) == 8U, "Unexpected EntryHeader layout");
    static_assert(sizeof(ObsoleteMarker) == 8U, "Unexpected ObsoleteMarker layout");

    static size_t batch_capacity();
    static size_t align8(size_t value);
    static size_t entry_size(size_t key_length, size_t value_length);
    static uint32_t checksum32(const uint8_t* data, size_t length, uint32_t hash = kHashSeed);
    static bool is_erased(uintptr_t address, size_t length);
    static void flash_clear_errors();
    static bool get_storage_page(size_t index, StoragePage& storage_page);
    static bool erase_storage_page(size_t index);
    // All callers supply erased, aligned doublewords and hold the Flash locks.
    static bool program_region(uintptr_t address, const uint8_t* data, size_t length);
    static bool read_page_header(size_t index, PageHeader& header);
    static bool choose_active_page(size_t& active_page, PageInfo& active_info);
    static bool validate_batch(const uint8_t* batch, size_t length);
    static bool scan_page(size_t index, PageInfo& info);
    // Used only on the validated committed prefix returned by scan_page().
    static bool next_record(size_t index, const PageInfo& info, RecordCursor& cursor, RecordView& record);
    static bool find_latest(size_t index, const PageInfo& info, const void* key, size_t key_length, RecordView& record);
    static bool append_entry(uint8_t* batch, size_t capacity, size_t& used, uint16_t& count,
                             const void* key, size_t key_length, const void* value, size_t value_length);
    static void finish_batch(uint8_t* batch, size_t used, uint16_t count);
    static bool program_batch(size_t index, size_t offset, const uint8_t* batch, size_t length);
    static bool retire_records(size_t index, const PageInfo& info);
    static bool take_mutex(SemaphoreHandle_t& mutex);
    static void give_mutex(SemaphoreHandle_t mutex);
    static bool take_locks();
    static void give_locks();
    static bool get_key_length(const char* key, size_t* length);
    static char* duplicate_key(const char* key, size_t length);

    Data* find_data(const void* key, size_t key_length) const;
    bool store_ram(const void* key, size_t key_length, const void* value, size_t length);
    bool load_page(size_t index, const PageInfo& info);
    bool compact_page(bool has_active_page, size_t active_page, uint32_t sequence, uint8_t* batch);
    bool fresh_locked();
    void clear_ram();

    Data* data_head_{nullptr};
    bool initialized_{false};
    bool append_failed_{false};
};
