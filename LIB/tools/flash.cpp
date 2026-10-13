#include "flash.hpp"

#include "main.h"

#include <cstdlib>
#include <cstring>
#include <limits>
#include <utility>

SemaphoreHandle_t Flash::write_mutex = nullptr;
SemaphoreHandle_t Flash::read_mutex = nullptr;

Flash::Flash() {
    (void)ensure_mutex(write_mutex);
    (void)ensure_mutex(read_mutex);
}

Flash::~Flash() {
    clear_ram();
}

void Flash::init() {
    if (!take_mutex(write_mutex)) {
        return;
    }
    if (!take_mutex(read_mutex)) {
        give_mutex(write_mutex);
        return;
    }

    clear_ram();

    size_t active_page = 0U;
    PageInfo active_info{};
    if (!choose_active_page(active_page, active_info)) {
        initialized_ = true;
        give_mutex(read_mutex);
        give_mutex(write_mutex);
        return;
    }

    uint8_t *blob = static_cast<uint8_t *>(std::malloc(blob_capacity()));
    if (blob == nullptr) {
        give_mutex(read_mutex);
        give_mutex(write_mutex);
        return;
    }

    size_t used = 0U;
    BlobHeader blob_header{};
    bool ok = read_page(active_page, &active_info, blob, blob_capacity()) &&
              validate_blob(blob, active_info.blob_size, &used) &&
              read_blob_header(blob, used, blob_header);

    if (ok) {
        data_.reserve(blob_header.entry_count);

        size_t offset = sizeof(BlobHeader);
        while (ok && offset < blob_header.used) {
            EntryHeader entry{};
            size_t entry_size = 0U;
            ok = read_entry_header(blob, blob_header.used, offset, entry) &&
                 entry_total_size(entry, &entry_size) &&
                 blob_header.used - offset >= entry_size &&
                 entry.value_length <= std::numeric_limits<int>::max();
            if (!ok) {
                break;
            }

            const char *key_begin =
                reinterpret_cast<const char *>(blob + offset + sizeof(EntryHeader));
            const uint8_t *value_begin =
                blob + offset + sizeof(EntryHeader) + entry.key_length;
            void *value = std::malloc(entry.value_length);
            if (value == nullptr) {
                ok = false;
                break;
            }

            std::memcpy(value, value_begin, entry.value_length);
            std::string key(key_begin, entry.key_length);
            auto inserted = data_.emplace(
                std::move(key),
                Data{value, static_cast<int>(entry.value_length)});
            if (!inserted.second) {
                std::free(inserted.first->second.p_data);
                inserted.first->second.p_data = value;
                inserted.first->second.size = static_cast<int>(entry.value_length);
            }

            offset += entry_size;
        }
    }

    std::free(blob);
    if (!ok) {
        clear_ram();
    }
    initialized_ = true;
    dirty_ = false;

    give_mutex(read_mutex);
    give_mutex(write_mutex);
}

void Flash::fresh() {
    if (!initialized_) {
        init();
    }
    if (!initialized_ || !take_mutex(write_mutex)) {
        return;
    }
    if (!take_mutex(read_mutex)) {
        give_mutex(write_mutex);
        return;
    }

    if (!dirty_) {
        give_mutex(read_mutex);
        give_mutex(write_mutex);
        return;
    }

    if (data_.size() > std::numeric_limits<uint16_t>::max()) {
        give_mutex(read_mutex);
        give_mutex(write_mutex);
        return;
    }

    const size_t capacity = blob_capacity();
    size_t used = sizeof(BlobHeader);
    uint16_t entry_count = 0U;
    bool ok = true;
    for (const auto &item : data_) {
        const Data &data = item.second;
        const size_t value_size = data.size < 0 ? 0U : static_cast<size_t>(data.size);
        if (item.first.empty() ||
            item.first.size() > std::numeric_limits<uint16_t>::max() ||
            data.p_data == nullptr ||
            data.size <= 0 ||
            value_size > std::numeric_limits<uint16_t>::max()) {
            ok = false;
            break;
        }

        const size_t entry_size = sizeof(EntryHeader) + item.first.size() + value_size;
        if (entry_size > std::numeric_limits<uint16_t>::max() ||
            used > capacity ||
            capacity - used < entry_size) {
            ok = false;
            break;
        }

        used += entry_size;
        ++entry_count;
    }

    uint8_t *blob = nullptr;
    if (ok) {
        blob = static_cast<uint8_t *>(std::malloc(used));
        ok = blob != nullptr;
    }

    if (ok) {
        BlobHeader blob_header{};
        blob_header.magic = kBlobMagic;
        blob_header.version = kBlobVersion;
        blob_header.header_size = sizeof(BlobHeader);
        blob_header.used = static_cast<uint16_t>(used);
        blob_header.entry_count = entry_count;
        blob_header.reserved = 0xFFFFFFFFUL;
        std::memcpy(blob, &blob_header, sizeof(blob_header));

        size_t offset = sizeof(BlobHeader);
        for (const auto &item : data_) {
            const Data &data = item.second;
            const size_t value_size = static_cast<size_t>(data.size);
            EntryHeader entry{};
            entry.key_length = static_cast<uint16_t>(item.first.size());
            entry.value_length = static_cast<uint16_t>(value_size);
            entry.reserved = 0xFFFFFFFFUL;

            std::memcpy(blob + offset, &entry, sizeof(entry));
            offset += sizeof(entry);
            std::memcpy(blob + offset, item.first.data(), item.first.size());
            offset += item.first.size();
            std::memcpy(blob + offset, data.p_data, value_size);
            offset += value_size;
        }

        if (offset != used || !validate_blob(blob, used, nullptr)) {
            ok = false;
        }
    }

    if (ok) {
        size_t active_page = 0U;
        PageInfo active_info{};
        const bool has_active_page = choose_active_page(active_page, active_info);
        const size_t target_page = has_active_page ? (1U - active_page) : 0U;
        const uint32_t next_sequence = has_active_page ? (active_info.sequence + 1U) : 1U;

        ok = erase_storage_page(target_page) &&
             program_page(target_page, blob, used, next_sequence);

        PageInfo written_info{};
        ok = ok &&
             read_page(target_page, &written_info, nullptr, blob_capacity()) &&
             written_info.sequence == next_sequence;

        if (ok && has_active_page) {
            (void)erase_storage_page(active_page);
        }
    }

    std::free(blob);
    if (ok) {
        dirty_ = false;
    }

    give_mutex(read_mutex);
    give_mutex(write_mutex);
}

void Flash::remove(std::string key) {
    if (!initialized_) {
        init();
    }
    if (!initialized_ || !take_mutex(write_mutex)) {
        return;
    }
    if (!take_mutex(read_mutex)) {
        give_mutex(write_mutex);
        return;
    }

    auto item = data_.find(key);
    if (item != data_.end()) {
        std::free(item->second.p_data);
        data_.erase(item);
        dirty_ = true;
    }

    give_mutex(read_mutex);
    give_mutex(write_mutex);
}

size_t Flash::storage_page_size() {
    return FLASH_PAGE_SIZE;
}

size_t Flash::blob_capacity() {
    const size_t page_size = storage_page_size();
    if (page_size <= kPageHeaderSize + kPageFooterSize) {
        return 0U;
    }
    return page_size - kPageHeaderSize - kPageFooterSize;
}

size_t Flash::align8(size_t value) {
    return (value + 7U) & ~size_t{7U};
}

uint32_t Flash::checksum32(const uint8_t *data, size_t length) {
    uint32_t hash = 2166136261UL;
    for (size_t i = 0U; i < length; ++i) {
        hash ^= data[i];
        hash *= 16777619UL;
    }
    return hash;
}

void Flash::flash_clear_errors() {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_ALL_ERRORS);
}

bool Flash::get_storage_page(size_t index, StoragePage &storage_page) {
    if (index >= kStoragePageCount || FLASH_SIZE < kStoragePageCount * storage_page_size()) {
        return false;
    }

    const uintptr_t address = FLASH_BASE + FLASH_SIZE -
                              ((kStoragePageCount - index) * storage_page_size());
    storage_page.address = address;

#if defined(FLASH_OPTR_DBANK)
    if (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK) != 0U &&
        address >= FLASH_BASE + FLASH_BANK_SIZE) {
        storage_page.bank = FLASH_BANK_2;
        storage_page.page = (address - (FLASH_BASE + FLASH_BANK_SIZE)) / storage_page_size();
    } else {
        storage_page.bank = FLASH_BANK_1;
        storage_page.page = (address - FLASH_BASE) / storage_page_size();
    }
#else
    storage_page.bank = FLASH_BANK_1;
    storage_page.page = (address - FLASH_BASE) / storage_page_size();
#endif

    return true;
}

bool Flash::erase_storage_page(size_t index) {
    StoragePage storage_page{};
    if (!get_storage_page(index, storage_page)) {
        return false;
    }

    FLASH_EraseInitTypeDef erase_init{};
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = storage_page.bank;
    erase_init.Page = storage_page.page;
    erase_init.NbPages = 1U;

    uint32_t page_error = 0U;
    HAL_FLASH_Unlock();
    flash_clear_errors();
    const bool ok = (HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK);
    HAL_FLASH_Lock();
    return ok;
}

bool Flash::program_region(uintptr_t address, const uint8_t *data, size_t length) {
    const size_t padded_length = align8(length);
    for (size_t offset = 0U; offset < padded_length; offset += sizeof(uint64_t)) {
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

bool Flash::program_page(size_t index,
                         const uint8_t *blob,
                         size_t blob_size,
                         uint32_t sequence) {
    StoragePage storage_page{};
    if (!get_storage_page(index, storage_page) || blob_size > blob_capacity()) {
        return false;
    }

    PageHeader header{};
    header.magic = kPageMagic;
    header.version = 1U;
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

bool Flash::read_page(size_t index, PageInfo *info, uint8_t *blob, size_t capacity) {
    StoragePage storage_page{};
    if (!get_storage_page(index, storage_page)) {
        return false;
    }

    PageHeader header{};
    std::memcpy(&header, reinterpret_cast<const void *>(storage_page.address), sizeof(header));

    if (header.magic != kPageMagic ||
        header.version != 1U ||
        header.header_size != sizeof(PageHeader) ||
        header.blob_size > blob_capacity() ||
        header.blob_size > capacity) {
        return false;
    }

    const size_t footer_offset = sizeof(PageHeader) + align8(header.blob_size);
    if (footer_offset + sizeof(PageFooter) > storage_page_size()) {
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

bool Flash::choose_active_page(size_t &active_page, PageInfo &active_info) {
    PageInfo info0{};
    PageInfo info1{};
    const bool page0_valid = read_page(0U, &info0, nullptr, blob_capacity());
    const bool page1_valid = read_page(1U, &info1, nullptr, blob_capacity());

    if (!page0_valid && !page1_valid) {
        return false;
    }

    if (page0_valid &&
        (!page1_valid ||
         (info0.sequence != info1.sequence && (info0.sequence - info1.sequence) < 0x80000000UL))) {
        active_page = 0U;
        active_info = info0;
    } else {
        active_page = 1U;
        active_info = info1;
    }
    return true;
}

bool Flash::read_blob_header(const uint8_t *blob, size_t length, BlobHeader &header) {
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

bool Flash::read_entry_header(const uint8_t *blob,
                              size_t used,
                              size_t offset,
                              EntryHeader &entry) {
    if (offset > used || used - offset < sizeof(EntryHeader)) {
        return false;
    }

    std::memcpy(&entry, blob + offset, sizeof(entry));
    return entry.key_length > 0U &&
           entry.value_length > 0U &&
           entry.reserved == 0xFFFFFFFFUL;
}

bool Flash::entry_total_size(const EntryHeader &entry, size_t *total_size) {
    const size_t total = sizeof(EntryHeader) + entry.key_length + entry.value_length;
    if (total > std::numeric_limits<uint16_t>::max()) {
        return false;
    }

    *total_size = total;
    return true;
}

bool Flash::validate_blob(const uint8_t *blob, size_t length, size_t *used) {
    BlobHeader header{};
    if (!read_blob_header(blob, length, header)) {
        return false;
    }

    size_t offset = sizeof(BlobHeader);
    uint16_t entry_count = 0U;
    while (offset < header.used) {
        EntryHeader entry{};
        size_t entry_size = 0U;
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

bool Flash::ensure_mutex(SemaphoreHandle_t &mutex) {
    if (mutex == nullptr) {
        mutex = xSemaphoreCreateMutex();
    }
    return mutex != nullptr;
}

bool Flash::take_mutex(SemaphoreHandle_t &mutex) {
    return ensure_mutex(mutex) && xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE;
}

void Flash::give_mutex(SemaphoreHandle_t mutex) {
    if (mutex != nullptr) {
        xSemaphoreGive(mutex);
    }
}

void Flash::clear_ram() {
    for (auto &item : data_) {
        std::free(item.second.p_data);
        item.second.p_data = nullptr;
        item.second.size = 0;
    }
    data_.clear();
    initialized_ = false;
    dirty_ = false;
}

bool Flash::write_raw(const std::string &key, const void *value, size_t length) {
    if (!initialized_) {
        init();
    }
    if (!initialized_ || !take_mutex(write_mutex)) {
        return false;
    }
    if (!take_mutex(read_mutex)) {
        give_mutex(write_mutex);
        return false;
    }

    bool ok = !key.empty() &&
              key.size() <= std::numeric_limits<uint16_t>::max() &&
              value != nullptr &&
              length > 0U &&
              length <= std::numeric_limits<uint16_t>::max() &&
              length <= static_cast<size_t>(std::numeric_limits<int>::max());

    if (ok) {
        const size_t capacity = blob_capacity();
        size_t projected_used = sizeof(BlobHeader);
        size_t projected_count = 0U;
        bool replacing = false;
        for (const auto &item : data_) {
            const bool same_key = item.first == key;
            const Data &data = item.second;
            const size_t value_size = same_key ? length : static_cast<size_t>(data.size);
            if (item.first.empty() ||
                item.first.size() > std::numeric_limits<uint16_t>::max() ||
                (!same_key && (data.p_data == nullptr || data.size <= 0)) ||
                value_size == 0U ||
                value_size > std::numeric_limits<uint16_t>::max()) {
                ok = false;
                break;
            }

            const size_t entry_size = sizeof(EntryHeader) + item.first.size() + value_size;
            if (entry_size > std::numeric_limits<uint16_t>::max() ||
                projected_used > capacity ||
                capacity - projected_used < entry_size) {
                ok = false;
                break;
            }

            projected_used += entry_size;
            ++projected_count;
            replacing = replacing || same_key;
        }

        if (ok && !replacing) {
            const size_t entry_size = sizeof(EntryHeader) + key.size() + length;
            ok = projected_count < std::numeric_limits<uint16_t>::max() &&
                 entry_size <= std::numeric_limits<uint16_t>::max() &&
                 projected_used <= capacity &&
                 capacity - projected_used >= entry_size;
        }
    }

    void *new_data = nullptr;
    if (ok) {
        new_data = std::malloc(length);
        ok = new_data != nullptr;
    }

    if (ok) {
        std::memcpy(new_data, value, length);
        auto item = data_.find(key);
        if (item != data_.end()) {
            std::free(item->second.p_data);
            item->second.p_data = new_data;
            item->second.size = static_cast<int>(length);
        } else {
            data_.reserve(data_.size() + 1U);
            auto inserted = data_.emplace(key, Data{new_data, static_cast<int>(length)});
            if (!inserted.second) {
                std::free(new_data);
                ok = false;
            }
        }
    }

    if (ok) {
        dirty_ = true;
    }

    give_mutex(read_mutex);
    give_mutex(write_mutex);
    return ok;
}

bool Flash::read_raw(const std::string &key, void *value, size_t length) {
    if (!initialized_) {
        init();
    }
    if (!initialized_ || !take_mutex(read_mutex)) {
        return false;
    }

    bool ok = !key.empty() &&
              value != nullptr &&
              length > 0U &&
              length <= static_cast<size_t>(std::numeric_limits<int>::max());
    if (ok) {
        auto item = data_.find(key);
        ok = item != data_.end() &&
             item->second.p_data != nullptr &&
             item->second.size == static_cast<int>(length);
        if (ok) {
            std::memcpy(value, item->second.p_data, length);
        }
    }

    give_mutex(read_mutex);
    return ok;
}
