#include "flash.hpp"

#include "main.h"

#include <cstdint>
#include <cstring>
#include <limits>

SemaphoreHandle_t Flash::write_mutex = nullptr;
SemaphoreHandle_t Flash::read_mutex = nullptr;

namespace {

constexpr size_t kStoragePageCount = 2;
constexpr size_t kStoragePageSize = FLASH_PAGE_SIZE;
constexpr size_t kPageHeaderSize = 32;
constexpr size_t kPageFooterSize = 8;
constexpr size_t kBlobCapacity = kStoragePageSize - kPageHeaderSize - kPageFooterSize;

constexpr uint32_t kPageMagic = 0x50464B56U;   // VKFP
constexpr uint32_t kFooterMagic = 0x54494D43U; // CMIT
constexpr uint32_t kBlobMagic = 0x3142564BU;   // KVB1
constexpr uint16_t kBlobVersion = 1;

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

static_assert(sizeof(PageHeader) == kPageHeaderSize, "PageHeader must stay flash-word aligned");
static_assert(sizeof(PageFooter) == kPageFooterSize, "PageFooter must stay flash-word aligned");
static_assert(sizeof(BlobHeader) == 16, "Unexpected BlobHeader layout");
static_assert(sizeof(EntryHeader) == 8, "Unexpected EntryHeader layout");

alignas(8) uint8_t blob_buffer[kBlobCapacity];

size_t align8(size_t value) {
    return (value + 7U) & ~size_t{7U};
}

uint32_t checksum32(const uint8_t *data, size_t length) {
    uint32_t hash = 2166136261UL;
    for (size_t i = 0; i < length; ++i) {
        hash ^= data[i];
        hash *= 16777619UL;
    }
    return hash;
}

void flash_clear_errors() {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_ALL_ERRORS);
}

bool get_storage_page(size_t index, StoragePage &storage_page) {
    if (index >= kStoragePageCount || FLASH_SIZE < kStoragePageCount * kStoragePageSize) {
        return false;
    }

    const uintptr_t address = FLASH_BASE + FLASH_SIZE -
                              ((kStoragePageCount - index) * kStoragePageSize);
    storage_page.address = address;

#if defined(FLASH_OPTR_DBANK)
    if (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK) != 0U &&
        address >= FLASH_BASE + FLASH_BANK_SIZE) {
        storage_page.bank = FLASH_BANK_2;
        storage_page.page = (address - (FLASH_BASE + FLASH_BANK_SIZE)) / kStoragePageSize;
    } else {
        storage_page.bank = FLASH_BANK_1;
        storage_page.page = (address - FLASH_BASE) / kStoragePageSize;
    }
#else
    storage_page.bank = FLASH_BANK_1;
    storage_page.page = (address - FLASH_BASE) / kStoragePageSize;
#endif

    return true;
}

bool erase_storage_page(size_t index) {
    StoragePage storage_page{};
    if (!get_storage_page(index, storage_page)) {
        return false;
    }

    FLASH_EraseInitTypeDef erase_init{};
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = storage_page.bank;
    erase_init.Page = storage_page.page;
    erase_init.NbPages = 1;

    uint32_t page_error = 0U;
    HAL_FLASH_Unlock();
    flash_clear_errors();
    const bool ok = (HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK);
    HAL_FLASH_Lock();
    return ok;
}

bool program_region(uintptr_t address, const uint8_t *data, size_t length) {
    const size_t padded_length = align8(length);
    for (size_t offset = 0; offset < padded_length; offset += sizeof(uint64_t)) {
        uint64_t word = 0xFFFFFFFFFFFFFFFFULL;
        const size_t remaining = (offset < length) ? (length - offset) : 0U;
        const size_t copy_length = (remaining < sizeof(word)) ? remaining : sizeof(word);
        if (copy_length > 0U) {
            std::memcpy(&word, data + offset, copy_length);
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              static_cast<uint32_t>(address + offset),
                              word) != HAL_OK) {
            return false;
        }
    }
    return true;
}

bool program_page(size_t index, const uint8_t *blob, size_t blob_size, uint32_t sequence) {
    StoragePage storage_page{};
    if (!get_storage_page(index, storage_page) || blob_size > kBlobCapacity) {
        return false;
    }

    PageHeader header{};
    header.magic = kPageMagic;
    header.version = 1;
    header.header_size = sizeof(PageHeader);
    header.sequence = sequence;
    header.blob_size = blob_size;
    header.blob_checksum = checksum32(blob, blob_size);
    header.reserved0 = 0xFFFFFFFFUL;
    header.reserved1 = 0xFFFFFFFFUL;
    header.reserved2 = 0xFFFFFFFFUL;

    PageFooter footer{};
    footer.magic = kFooterMagic;
    footer.sequence = sequence;

    HAL_FLASH_Unlock();
    flash_clear_errors();

    bool ok = program_region(storage_page.address,
                             reinterpret_cast<const uint8_t *>(&header),
                             sizeof(header));
    ok = ok && program_region(storage_page.address + sizeof(header), blob, blob_size);
    ok = ok && program_region(storage_page.address + sizeof(header) + align8(blob_size),
                              reinterpret_cast<const uint8_t *>(&footer),
                              sizeof(footer));

    HAL_FLASH_Lock();
    return ok;
}

bool read_page(size_t index, PageInfo *info, uint8_t *blob, size_t capacity) {
    StoragePage storage_page{};
    if (!get_storage_page(index, storage_page)) {
        return false;
    }

    PageHeader header{};
    std::memcpy(&header, reinterpret_cast<const void *>(storage_page.address), sizeof(header));

    if (header.magic != kPageMagic ||
        header.version != 1 ||
        header.header_size != sizeof(PageHeader) ||
        header.blob_size > kBlobCapacity ||
        header.blob_size > capacity) {
        return false;
    }

    const size_t footer_offset = sizeof(PageHeader) + align8(header.blob_size);
    if (footer_offset + sizeof(PageFooter) > kStoragePageSize) {
        return false;
    }

    PageFooter footer{};
    std::memcpy(&footer,
                reinterpret_cast<const void *>(storage_page.address + footer_offset),
                sizeof(footer));

    if (footer.magic != kFooterMagic || footer.sequence != header.sequence) {
        return false;
    }

    const auto *flash_blob =
        reinterpret_cast<const uint8_t *>(storage_page.address + sizeof(PageHeader));
    if (checksum32(flash_blob, header.blob_size) != header.blob_checksum) {
        return false;
    }

    if (blob != nullptr && header.blob_size > 0U) {
        std::memcpy(blob, flash_blob, header.blob_size);
    }
    if (info != nullptr) {
        info->sequence = header.sequence;
        info->blob_size = header.blob_size;
    }
    return true;
}

bool sequence_is_newer(uint32_t lhs, uint32_t rhs) {
    return lhs != rhs && (lhs - rhs) < 0x80000000UL;
}

bool choose_active_page(size_t &active_page, PageInfo &active_info) {
    PageInfo info0{};
    PageInfo info1{};
    const bool page0_valid = read_page(0, &info0, nullptr, kBlobCapacity);
    const bool page1_valid = read_page(1, &info1, nullptr, kBlobCapacity);

    if (!page0_valid && !page1_valid) {
        return false;
    }

    if (page0_valid && (!page1_valid || sequence_is_newer(info0.sequence, info1.sequence))) {
        active_page = 0;
        active_info = info0;
    } else {
        active_page = 1;
        active_info = info1;
    }
    return true;
}

void write_blob_header(uint8_t *blob, uint16_t used, uint16_t entry_count) {
    BlobHeader header{};
    header.magic = kBlobMagic;
    header.version = kBlobVersion;
    header.header_size = sizeof(BlobHeader);
    header.used = used;
    header.entry_count = entry_count;
    header.reserved = 0xFFFFFFFFUL;
    std::memcpy(blob, &header, sizeof(header));
}

void init_empty_blob(uint8_t *blob, size_t *used) {
    std::memset(blob, 0xFF, kBlobCapacity);
    write_blob_header(blob, sizeof(BlobHeader), 0);
    if (used != nullptr) {
        *used = sizeof(BlobHeader);
    }
}

bool read_blob_header(const uint8_t *blob, size_t length, BlobHeader &header) {
    if (length < sizeof(BlobHeader)) {
        return false;
    }
    std::memcpy(&header, blob, sizeof(header));
    return header.magic == kBlobMagic &&
           header.version == kBlobVersion &&
           header.header_size == sizeof(BlobHeader) &&
           header.used >= sizeof(BlobHeader) &&
           header.used <= length;
}

bool read_entry_header(const uint8_t *blob, size_t used, size_t offset, EntryHeader &entry) {
    if (offset > used || used - offset < sizeof(EntryHeader)) {
        return false;
    }
    std::memcpy(&entry, blob + offset, sizeof(entry));
    return entry.key_length > 0U &&
           entry.value_length > 0U &&
           entry.reserved == 0xFFFFFFFFUL;
}

bool entry_total_size(const EntryHeader &entry, size_t *total_size) {
    const size_t total = sizeof(EntryHeader) + entry.key_length + entry.value_length;
    if (total > std::numeric_limits<uint16_t>::max()) {
        return false;
    }
    *total_size = total;
    return true;
}

bool validate_blob(const uint8_t *blob, size_t length, size_t *used) {
    BlobHeader header{};
    if (!read_blob_header(blob, length, header)) {
        return false;
    }

    size_t offset = sizeof(BlobHeader);
    uint16_t entry_count = 0;
    while (offset < header.used) {
        EntryHeader entry{};
        size_t entry_size = 0;
        if (!read_entry_header(blob, header.used, offset, entry) ||
            !entry_total_size(entry, &entry_size) ||
            header.used - offset < entry_size) {
            return false;
        }

        offset += entry_size;
        ++entry_count;
    }

    if (offset != header.used || entry_count != header.entry_count) {
        return false;
    }

    if (used != nullptr) {
        *used = header.used;
    }
    return true;
}

bool load_blob(uint8_t *blob, size_t capacity, size_t *used) {
    size_t active_page = 0;
    PageInfo active_info{};
    if (!choose_active_page(active_page, active_info)) {
        return false;
    }

    if (!read_page(active_page, &active_info, blob, capacity)) {
        return false;
    }

    return validate_blob(blob, active_info.blob_size, used);
}

bool save_blob(const uint8_t *blob, size_t used) {
    if (used > kBlobCapacity || !validate_blob(blob, used, nullptr)) {
        return false;
    }

    size_t active_page = 0;
    PageInfo active_info{};
    const bool has_active_page = choose_active_page(active_page, active_info);
    const size_t target_page = has_active_page ? (1U - active_page) : 0U;
    const uint32_t next_sequence = has_active_page ? (active_info.sequence + 1U) : 1U;

    if (!erase_storage_page(target_page) ||
        !program_page(target_page, blob, used, next_sequence)) {
        return false;
    }

    PageInfo written_info{};
    if (!read_page(target_page, &written_info, nullptr, kBlobCapacity) ||
        written_info.sequence != next_sequence) {
        return false;
    }

    if (has_active_page) {
        (void)erase_storage_page(active_page);
    }

    return true;
}

bool put_entry(uint8_t *blob,
               size_t capacity,
               size_t current_used,
               const std::string &key,
               const void *value,
               size_t value_length,
               size_t *new_used) {
    if (key.empty() ||
        key.size() > std::numeric_limits<uint16_t>::max() ||
        value == nullptr ||
        value_length == 0U ||
        value_length > std::numeric_limits<uint16_t>::max()) {
        return false;
    }

    BlobHeader header{};
    if (!read_blob_header(blob, current_used, header)) {
        return false;
    }

    size_t read_offset = sizeof(BlobHeader);
    size_t write_offset = sizeof(BlobHeader);
    uint16_t entry_count = 0;

    while (read_offset < header.used) {
        EntryHeader entry{};
        size_t entry_size = 0;
        if (!read_entry_header(blob, header.used, read_offset, entry) ||
            !entry_total_size(entry, &entry_size) ||
            header.used - read_offset < entry_size) {
            return false;
        }

        const uint8_t *entry_key = blob + read_offset + sizeof(EntryHeader);
        const bool same_key =
            entry.key_length == key.size() &&
            std::memcmp(entry_key, key.data(), key.size()) == 0;

        if (!same_key) {
            if (write_offset != read_offset) {
                std::memmove(blob + write_offset, blob + read_offset, entry_size);
            }
            write_offset += entry_size;
            ++entry_count;
        }

        read_offset += entry_size;
    }

    const size_t entry_size = sizeof(EntryHeader) + key.size() + value_length;
    if (capacity - write_offset < entry_size) {
        return false;
    }

    EntryHeader entry{};
    entry.key_length = static_cast<uint16_t>(key.size());
    entry.value_length = static_cast<uint16_t>(value_length);
    entry.reserved = 0xFFFFFFFFUL;

    std::memcpy(blob + write_offset, &entry, sizeof(entry));
    write_offset += sizeof(entry);
    std::memcpy(blob + write_offset, key.data(), key.size());
    write_offset += key.size();
    std::memcpy(blob + write_offset, value, value_length);
    write_offset += value_length;
    ++entry_count;

    if (write_offset > std::numeric_limits<uint16_t>::max()) {
        return false;
    }

    write_blob_header(blob, static_cast<uint16_t>(write_offset), entry_count);
    if (new_used != nullptr) {
        *new_used = write_offset;
    }
    return true;
}

bool get_entry(const uint8_t *blob,
               size_t used,
               const std::string &key,
               void *value,
               size_t value_length) {
    if (key.empty() || value == nullptr || value_length == 0U) {
        return false;
    }

    BlobHeader header{};
    if (!read_blob_header(blob, used, header)) {
        return false;
    }

    size_t offset = sizeof(BlobHeader);
    while (offset < header.used) {
        EntryHeader entry{};
        size_t entry_size = 0;
        if (!read_entry_header(blob, header.used, offset, entry) ||
            !entry_total_size(entry, &entry_size) ||
            header.used - offset < entry_size) {
            return false;
        }

        const uint8_t *entry_key = blob + offset + sizeof(EntryHeader);
        const uint8_t *entry_value = entry_key + entry.key_length;
        const bool same_key =
            entry.key_length == key.size() &&
            std::memcmp(entry_key, key.data(), key.size()) == 0;

        if (same_key) {
            if (entry.value_length != value_length) {
                return false;
            }
            std::memcpy(value, entry_value, value_length);
            return true;
        }

        offset += entry_size;
    }

    return false;
}

bool ensure_mutex(SemaphoreHandle_t &mutex) {
    if (mutex == nullptr) {
        mutex = xSemaphoreCreateMutex();
    }
    return mutex != nullptr;
}

bool take_mutex(SemaphoreHandle_t &mutex) {
    return ensure_mutex(mutex) && xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE;
}

void give_mutex(SemaphoreHandle_t mutex) {
    if (mutex != nullptr) {
        xSemaphoreGive(mutex);
    }
}

} // namespace

Flash::Flash() {
    (void)ensure_mutex(write_mutex);
    (void)ensure_mutex(read_mutex);
}

bool Flash::write_raw(const std::string &key, const void *value, size_t length) {
    if (!take_mutex(write_mutex)) {
        return false;
    }
    if (!take_mutex(read_mutex)) {
        give_mutex(write_mutex);
        return false;
    }

    size_t used = 0;
    if (!load_blob(blob_buffer, sizeof(blob_buffer), &used)) {
        init_empty_blob(blob_buffer, &used);
    }

    bool ok = put_entry(blob_buffer, sizeof(blob_buffer), used, key, value, length, &used);
    ok = ok && save_blob(blob_buffer, used);

    give_mutex(read_mutex);
    give_mutex(write_mutex);
    return ok;
}

bool Flash::read_raw(const std::string &key, void *value, size_t length) {
    if (!take_mutex(read_mutex)) {
        return false;
    }

    size_t used = 0;
    const bool ok = load_blob(blob_buffer, sizeof(blob_buffer), &used) &&
                    get_entry(blob_buffer, used, key, value, length);

    give_mutex(read_mutex);
    return ok;
}
