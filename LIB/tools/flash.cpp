#include "flash.hpp"
#include "stm32g4xx_hal.h"

#include <cstdlib>
#include <cstring>
#include <new>

SemaphoreHandle_t Flash::write_mutex = nullptr;
SemaphoreHandle_t Flash::read_mutex = nullptr;

Flash::Flash() = default;

Flash::~Flash() {
    clear_ram();
}

void Flash::init() {
    // One task initializes the object before publishing it to other tasks.
    if (!take_locks()) {
        return;
    }
    clear_ram();

    size_t active_page = 0U;
    PageInfo info{};
    bool reset = false;
    if (choose_active_page(active_page, info)) {
        if (!scan_page(active_page, info)) {
            reset = true;
        } else if (load_page(active_page, info)) {
            initialized_ = true;
        } else {
            // Allocation failure is not a format error. Keep Flash for retry.
            clear_ram();
        }
    } else {
        StoragePage page0{};
        StoragePage page1{};
        if (get_storage_page(0U, page0) && get_storage_page(1U, page1)) {
            reset = !is_erased(page0.address, FLASH_PAGE_SIZE) || !is_erased(page1.address, FLASH_PAGE_SIZE);
            initialized_ = !reset;
        }
    }

    give_locks();
    if (reset) {
        // clear() takes the same non-recursive locks. Do not call it while locked.
        (void)clear();
    }
}

bool Flash::fresh() {
    if (!initialized_) {
        init();
    }
    if (!initialized_ || !take_locks()) {
        return false;
    }
    const bool ok = fresh_locked();
    give_locks();
    return ok;
}

bool Flash::fresh_locked() {
    size_t active_page = 0U;
    PageInfo info{};
    const bool has_active_page = choose_active_page(active_page, info);
    if (has_active_page && !scan_page(active_page, info)) {
        // Runtime corruption must not silently erase the caller's RAM changes.
        return false;
    }

    const size_t capacity = batch_capacity();
    auto* batch = static_cast<uint8_t*>(std::malloc(capacity));
    if (batch == nullptr) {
        return false;
    }
    size_t used = sizeof(BatchHeader);
    uint16_t count = 0U;
    bool fits = true;
    for (const Data* item = data_head_; item != nullptr && fits; item = item->next) {
        RecordView stored{};
        const bool same = has_active_page && find_latest(active_page, info, item->key, item->key_length, stored)
                          && stored.header.type == kSetEntry && stored.header.value_length == item->size
                          && std::memcmp(stored.value, item->p_data, item->size) == 0;
        if (!same) {
            fits = append_entry(batch, capacity, used, count, item->key, item->key_length, item->p_data, item->size);
        }
    }

    if (has_active_page && fits) {
        RecordCursor cursor{kPagePrefixSize, 0U};
        RecordView record{};
        while (fits && next_record(active_page, info, cursor, record)) {
            if (record.header.type != kSetEntry || find_data(record.key, record.header.key_length) != nullptr) {
                continue;
            }
            RecordView latest{};
            if (find_latest(active_page, info, record.key, record.header.key_length, latest)
                && latest.obsolete_address == record.obsolete_address) {
                fits = append_entry(batch, capacity, used, count, record.key, record.header.key_length, nullptr, 0U);
            }
        }
    }

    bool ok = true;
    if (fits && count == 0U) {
        // Includes writes of identical values, reverting a value, and add/remove
        // pairs that were never persisted. No Flash programming or erase occurs.
        std::free(batch);
        return true;
    }
    const size_t total_size = used + sizeof(BatchCommit);
    if (fits && has_active_page && info.appendable && !append_failed_
        && total_size <= FLASH_PAGE_SIZE - info.committed_end) {
        finish_batch(batch, used, count);
        ok = program_batch(active_page, info.committed_end, batch, total_size);
        PageInfo written{};
        ok = ok && scan_page(active_page, written) && written.committed_end == info.committed_end + total_size;
        if (ok) {
            // The new batch is already durable. Failure to retire old records
            // cannot roll it back; log order resolves duplicates on reboot.
            append_failed_ = !retire_records(active_page, written);
        } else {
            // A failed or interrupted doubleword must never be programmed again.
            append_failed_ = true;
        }
    } else {
        ok = compact_page(has_active_page, active_page, has_active_page ? info.sequence + 1U : 1U, batch);
    }
    std::free(batch);
    return ok;
}

bool Flash::compact_page(bool has_active_page, size_t active_page, uint32_t sequence, uint8_t* batch) {
    size_t used = sizeof(BatchHeader);
    uint16_t count = 0U;
    for (const Data* item = data_head_; item != nullptr; item = item->next) {
        if (!append_entry(batch, batch_capacity(), used, count, item->key, item->key_length, item->p_data, item->size)) {
            return false;
        }
    }
    finish_batch(batch, used, count);
    const size_t total_size = used + sizeof(BatchCommit);
    const size_t target_page = has_active_page ? 1U - active_page : 0U;
    StoragePage target{};
    if (!get_storage_page(target_page, target)) {
        return false;
    }
    // Keep the previous page until the next collection needs it. Always erase
    // the target, even if it reads all ones: a power cut may have consumed ECC
    // bits without visibly changing a doubleword. Never reuse such a target.
    if (!erase_storage_page(target_page)) {
        return false;
    }

    PageHeader header{kPageMagic, kFormatVersion, kPagePrefixSize, sequence, 0U};
    header.checksum = checksum32(reinterpret_cast<const uint8_t*>(&header), offsetof(PageHeader, checksum));
    HAL_FLASH_Unlock();
    flash_clear_errors();
    bool ok = program_region(target.address, reinterpret_cast<const uint8_t*>(&header), sizeof(header));
    HAL_FLASH_Lock();
    ok = ok && program_batch(target_page, kPagePrefixSize, batch, total_size);
    if (!ok) {
        return false;
    }

    // The immutable page header and initial committed batch have been read back
    // before this final, independent activation doubleword is programmed.
    const PageActivation activation{kActivationMagic, sequence};
    HAL_FLASH_Unlock();
    flash_clear_errors();
    ok = program_region(target.address + sizeof(PageHeader), reinterpret_cast<const uint8_t*>(&activation), sizeof(activation));
    HAL_FLASH_Lock();
    PageInfo written{};
    ok = ok && scan_page(target_page, written) && written.sequence == sequence
         && written.committed_end == kPagePrefixSize + total_size;
    if (ok) {
        append_failed_ = false;
    }
    return ok;
}

bool Flash::clear() {
    if (!take_locks()) {
        return false;
    }
    bool ok = true;
    for (size_t page = 0U; page < kStoragePageCount; ++page) {
        if (!erase_storage_page(page)) {
            ok = false;
            break;
        }
    }
    if (ok) {
        clear_ram();
        initialized_ = true;
    }
    give_locks();
    return ok;
}

void Flash::remove(const char* key) {
    if (!initialized_) {
        init();
    }
    if (!initialized_ || !take_locks()) {
        return;
    }
    size_t key_size = 0U;
    if (get_key_length(key, &key_size)) {
        Data** link = &data_head_;
        while (*link != nullptr && std::strcmp((*link)->key, key) != 0) {
            link = &(*link)->next;
        }
        if (*link != nullptr) {
            Data* item = *link;
            *link = item->next;
            std::free(item->p_data);
            std::free(item->key);
            delete item;
        }
    }
    give_locks();
}

bool Flash::get_key_length(const char* key, size_t* length) {
    if (key == nullptr || key[0] == '\0') {
        return false;
    }
    const size_t key_size = std::strlen(key);
    if (key_size > UINT16_MAX) {
        return false;
    }
    if (length != nullptr) {
        *length = key_size;
    }
    return true;
}

char* Flash::duplicate_key(const char* key, size_t length) {
    // Both callers supply keys validated by write_raw() or validate_batch().
    char* copy = static_cast<char*>(std::malloc(length + 1U));
    if (copy != nullptr) {
        std::memcpy(copy, key, length);
        copy[length] = '\0';
    }
    return copy;
}

void Flash::clear_ram() {
    while (data_head_ != nullptr) {
        Data* item = data_head_;
        data_head_ = item->next;
        std::free(item->key);
        std::free(item->p_data);
        delete item;
    }
    initialized_ = false;
    append_failed_ = false;
}

size_t Flash::batch_capacity() {
    return FLASH_PAGE_SIZE - kPagePrefixSize;
}

size_t Flash::align8(size_t value) {
    return (value + 7U) & ~size_t{7U};
}

size_t Flash::entry_size(size_t key_length, size_t value_length) {
    return sizeof(EntryHeader) + align8(key_length + value_length) + sizeof(ObsoleteMarker);
}

uint32_t Flash::checksum32(const uint8_t* data, size_t length, uint32_t hash) {
    for (size_t i = 0U; i < length; ++i) {
        hash ^= data[i];
        hash *= 16777619UL;
    }
    return hash;
}

bool Flash::is_erased(uintptr_t address, size_t length) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(address);
    for (size_t i = 0U; i < length; ++i) {
        if (bytes[i] != 0xFFU) {
            return false;
        }
    }
    return true;
}

void Flash::flash_clear_errors() {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_ALL_ERRORS);
}

bool Flash::get_storage_page(size_t index, StoragePage& storage_page) {
    const uint32_t flash_size = FLASH_SIZE;
    if (index >= kStoragePageCount || flash_size < kStoragePageCount * FLASH_PAGE_SIZE) {
        return false;
    }
    const uintptr_t address = FLASH_BASE + flash_size - ((kStoragePageCount - index) * FLASH_PAGE_SIZE);
    storage_page.address = address;
#if defined(FLASH_OPTR_DBANK)
    if (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK) != 0U && address >= FLASH_BASE + FLASH_BANK_SIZE) {
        storage_page.bank = FLASH_BANK_2;
        storage_page.page = (address - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
    } else {
        storage_page.bank = FLASH_BANK_1;
        storage_page.page = (address - FLASH_BASE) / FLASH_PAGE_SIZE;
    }
#else
    storage_page.bank = FLASH_BANK_1;
    storage_page.page = (address - FLASH_BASE) / FLASH_PAGE_SIZE;
#endif
    return true;
}

bool Flash::erase_storage_page(size_t index) {
    StoragePage page{};
    if (!get_storage_page(index, page)) {
        return false;
    }
    FLASH_EraseInitTypeDef erase_init{};
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = page.bank;
    erase_init.Page = page.page;
    erase_init.NbPages = 1U;
    uint32_t page_error = 0U;
    HAL_FLASH_Unlock();
    flash_clear_errors();
    const bool ok = HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK;
    HAL_FLASH_Lock();
    return ok && is_erased(page.address, FLASH_PAGE_SIZE);
}

bool Flash::program_region(uintptr_t address, const uint8_t* data, size_t length) {
    for (size_t offset = 0U; offset < align8(length); offset += sizeof(uint64_t)) {
        uint64_t word = UINT64_MAX;
        const size_t remaining = length - offset;
        const size_t copy_length = remaining < sizeof(word) ? remaining : sizeof(word);
        std::memcpy(&word, data + offset, copy_length);
        // Programming all ones still consumes ECC bits. Leave such words erased.
        if (word != UINT64_MAX
            && HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, static_cast<uint32_t>(address + offset), word) != HAL_OK) {
            return false;
        }
        if (std::memcmp(reinterpret_cast<const void*>(address + offset), &word, sizeof(word)) != 0) {
            return false;
        }
    }
    return true;
}

bool Flash::read_page_header(size_t index, PageHeader& header) {
    StoragePage page{};
    if (!get_storage_page(index, page)) {
        return false;
    }
    std::memcpy(&header, reinterpret_cast<const void*>(page.address), sizeof(header));
    if (header.magic != kPageMagic || header.version != kFormatVersion || header.header_size != kPagePrefixSize
        || header.checksum != checksum32(reinterpret_cast<const uint8_t*>(&header), offsetof(PageHeader, checksum))) {
        return false;
    }
    PageActivation activation{};
    std::memcpy(&activation, reinterpret_cast<const void*>(page.address + sizeof(PageHeader)), sizeof(activation));
    return activation.magic == kActivationMagic && activation.sequence == header.sequence;
}

bool Flash::choose_active_page(size_t& active_page, PageInfo& active_info) {
    PageHeader headers[kStoragePageCount]{};
    const bool valid0 = read_page_header(0U, headers[0]);
    const bool valid1 = read_page_header(1U, headers[1]);
    if (!valid0 && !valid1) {
        return false;
    }
    active_page = valid0 && (!valid1 || (headers[0].sequence != headers[1].sequence
                  && (headers[0].sequence - headers[1].sequence) < 0x80000000UL)) ? 0U : 1U;
    active_info = {headers[active_page].sequence, kPagePrefixSize, true};
    return true;
}

bool Flash::validate_batch(const uint8_t* batch, size_t length) {
    if (length < kBatchOverhead || length > batch_capacity() || (length & 7U) != 0U) {
        return false;
    }
    BatchHeader header{};
    std::memcpy(&header, batch, sizeof(header));
    if (header.magic != kBatchMagic || header.total_size != length || header.reserved != UINT32_MAX) {
        return false;
    }
    size_t offset = sizeof(BatchHeader);
    const size_t end = length - sizeof(BatchCommit);
    uint32_t hash = checksum32(batch, offsetof(BatchHeader, checksum));
    for (uint16_t i = 0U; i < header.entry_count; ++i) {
        if (offset > end || end - offset < sizeof(EntryHeader)) {
            return false;
        }
        EntryHeader entry{};
        std::memcpy(&entry, batch + offset, sizeof(entry));
        if (entry.key_length == 0U || (entry.type != kSetEntry && entry.type != kDeleteEntry)
            || (entry.type == kSetEntry ? entry.value_length == 0U : entry.value_length != 0U)
            || entry.total_size != entry_size(entry.key_length, entry.value_length) || end - offset < entry.total_size) {
            return false;
        }
        if (std::memchr(batch + offset + sizeof(EntryHeader), '\0', entry.key_length) != nullptr) {
            return false;
        }
        hash = checksum32(batch + offset, entry.total_size - sizeof(ObsoleteMarker), hash);
        offset += entry.total_size;
    }
    BatchCommit commit{};
    std::memcpy(&commit, batch + end, sizeof(commit));
    return offset == end && commit.magic == kCommitMagic && commit.inverse == ~kCommitMagic && header.checksum == hash;
}

bool Flash::scan_page(size_t index, PageInfo& info) {
    PageHeader header{};
    StoragePage page{};
    if (!read_page_header(index, header) || !get_storage_page(index, page)) {
        return false;
    }
    info = {header.sequence, kPagePrefixSize, true};
    size_t offset = kPagePrefixSize;
    while (offset < FLASH_PAGE_SIZE) {
        const size_t remaining = FLASH_PAGE_SIZE - offset;
        if (is_erased(page.address + offset, remaining)) {
            break;
        }
        BatchHeader batch{};
        if (remaining < kBatchOverhead) {
            info.appendable = false;
            break;
        }
        std::memcpy(&batch, reinterpret_cast<const void*>(page.address + offset), sizeof(batch));
        if (batch.magic != kBatchMagic || batch.total_size < kBatchOverhead || batch.total_size > remaining
            || (batch.total_size & 7U) != 0U) {
            info.appendable = false;
            break;
        }
        BatchCommit commit{};
        std::memcpy(&commit, reinterpret_cast<const void*>(page.address + offset + batch.total_size - sizeof(commit)), sizeof(commit));
        if (commit.magic != kCommitMagic || commit.inverse != ~kCommitMagic) {
            info.appendable = false;
            break;
        }
        if (!validate_batch(reinterpret_cast<const uint8_t*>(page.address + offset), batch.total_size)) {
            // A committed batch with an invalid schema/checksum is corruption,
            // not an interrupted append. init() resets incompatible storage.
            return false;
        }
        offset += batch.total_size;
        info.committed_end = offset;
    }
    // An activated page always contains at least its initial committed batch.
    return info.committed_end > kPagePrefixSize;
}

bool Flash::next_record(size_t index, const PageInfo& info, RecordCursor& cursor, RecordView& record) {
    StoragePage page{};
    if (cursor.offset >= info.committed_end || !get_storage_page(index, page)) {
        return false;
    }
    while (cursor.remaining == 0U) {
        BatchHeader batch{};
        std::memcpy(&batch, reinterpret_cast<const void*>(page.address + cursor.offset), sizeof(batch));
        cursor.offset += sizeof(batch);
        cursor.remaining = batch.entry_count;
        if (cursor.remaining == 0U) {
            cursor.offset += sizeof(BatchCommit);
            if (cursor.offset >= info.committed_end) {
                return false;
            }
        }
    }
    std::memcpy(&record.header, reinterpret_cast<const void*>(page.address + cursor.offset), sizeof(record.header));
    record.key = reinterpret_cast<const uint8_t*>(page.address + cursor.offset + sizeof(EntryHeader));
    record.value = record.key + record.header.key_length;
    record.obsolete_address = page.address + cursor.offset + record.header.total_size - sizeof(ObsoleteMarker);
    cursor.offset += record.header.total_size;
    if (--cursor.remaining == 0U) {
        cursor.offset += sizeof(BatchCommit);
    }
    return true;
}

bool Flash::find_latest(size_t index, const PageInfo& info, const void* key, size_t key_length, RecordView& record) {
    RecordCursor cursor{kPagePrefixSize, 0U};
    RecordView candidate{};
    bool found = false;
    while (next_record(index, info, cursor, candidate)) {
        if (candidate.header.key_length == key_length && std::memcmp(candidate.key, key, key_length) == 0) {
            record = candidate;
            found = true;
        }
    }
    return found;
}

bool Flash::append_entry(uint8_t* batch, size_t capacity, size_t& used, uint16_t& count,
                         const void* key, size_t key_length, const void* value, size_t value_length) {
    const size_t total = entry_size(key_length, value_length);
    // Keys/values have already been validated. Fitting in one 2 KiB page also
    // guarantees that all serialized lengths and the entry count fit uint16_t.
    if (used > capacity || capacity - used < total + sizeof(BatchCommit)) {
        return false;
    }
    const EntryHeader entry{static_cast<uint16_t>(key_length), static_cast<uint16_t>(value_length),
                            value_length == 0U ? kDeleteEntry : kSetEntry, static_cast<uint16_t>(total)};
    std::memset(batch + used, 0xFF, total);
    std::memcpy(batch + used, &entry, sizeof(entry));
    std::memcpy(batch + used + sizeof(entry), key, key_length);
    if (value_length != 0U) {
        std::memcpy(batch + used + sizeof(entry) + key_length, value, value_length);
    }
    used += total;
    ++count;
    return true;
}

void Flash::finish_batch(uint8_t* batch, size_t used, uint16_t count) {
    BatchHeader header{kBatchMagic, static_cast<uint16_t>(used + sizeof(BatchCommit)), count, 0U, UINT32_MAX};
    std::memcpy(batch, &header, sizeof(header));
    uint32_t hash = checksum32(batch, offsetof(BatchHeader, checksum));
    size_t offset = sizeof(header);
    for (uint16_t i = 0U; i < count; ++i) {
        EntryHeader entry{};
        std::memcpy(&entry, batch + offset, sizeof(entry));
        hash = checksum32(batch + offset, entry.total_size - sizeof(ObsoleteMarker), hash);
        offset += entry.total_size;
    }
    header.checksum = hash;
    std::memcpy(batch, &header, sizeof(header));
    const BatchCommit commit{kCommitMagic, ~kCommitMagic};
    std::memcpy(batch + used, &commit, sizeof(commit));
}

bool Flash::program_batch(size_t index, size_t offset, const uint8_t* batch, size_t length) {
    StoragePage page{};
    if (!get_storage_page(index, page) || offset > FLASH_PAGE_SIZE || length > FLASH_PAGE_SIZE - offset
        || !validate_batch(batch, length)) {
        return false;
    }
    BatchHeader header{};
    std::memcpy(&header, batch, sizeof(header));
    HAL_FLASH_Unlock();
    flash_clear_errors();
    bool ok = program_region(page.address + offset, batch, sizeof(header));
    size_t position = sizeof(header);
    for (uint16_t i = 0U; ok && i < header.entry_count; ++i) {
        EntryHeader entry{};
        std::memcpy(&entry, batch + position, sizeof(entry));
        ok = program_region(page.address + offset + position, batch + position, entry.total_size - sizeof(ObsoleteMarker));
        // Skip the whole erased marker doubleword, including its ECC bits.
        position += entry.total_size;
    }
    // Read back every immutable record before publishing the batch. Comparing
    // the erased markers also detects an accidentally reused write location.
    ok = ok && std::memcmp(reinterpret_cast<const void*>(page.address + offset), batch, length - sizeof(BatchCommit)) == 0;
    ok = ok && program_region(page.address + offset + length - sizeof(BatchCommit),
                             batch + length - sizeof(BatchCommit), sizeof(BatchCommit));
    HAL_FLASH_Lock();
    return ok && validate_batch(reinterpret_cast<const uint8_t*>(page.address + offset), length);
}

bool Flash::retire_records(size_t index, const PageInfo& info) {
    RecordCursor cursor{kPagePrefixSize, 0U};
    RecordView record{};
    bool ok = true;
    HAL_FLASH_Unlock();
    flash_clear_errors();
    while (ok && next_record(index, info, cursor, record)) {
        RecordView latest{};
        if (find_latest(index, info, record.key, record.header.key_length, latest)
            && latest.obsolete_address != record.obsolete_address
            && is_erased(record.obsolete_address, sizeof(ObsoleteMarker))) {
            const ObsoleteMarker marker{kObsoleteMagic, ~kObsoleteMagic};
            ok = program_region(record.obsolete_address, reinterpret_cast<const uint8_t*>(&marker), sizeof(marker));
        }
    }
    HAL_FLASH_Lock();
    return ok;
}

bool Flash::take_mutex(SemaphoreHandle_t& mutex) {
    if (mutex == nullptr) {
        mutex = xSemaphoreCreateMutex();
    }
    return mutex != nullptr && xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE;
}

void Flash::give_mutex(SemaphoreHandle_t mutex) {
    // Only called after a successful take; mutex is always non-null here.
    xSemaphoreGive(mutex);
}

bool Flash::take_locks() {
    if (!take_mutex(write_mutex)) {
        return false;
    }
    if (!take_mutex(read_mutex)) {
        give_mutex(write_mutex);
        return false;
    }
    return true;
}

void Flash::give_locks() {
    give_mutex(read_mutex);
    give_mutex(write_mutex);
}

Flash::Data* Flash::find_data(const void* key, size_t key_length) const {
    Data* item = data_head_;
    while (item != nullptr) {
        if (item->key_length == key_length && std::memcmp(item->key, key, key_length) == 0) {
            return item;
        }
        item = item->next;
    }
    return nullptr;
}

bool Flash::store_ram(const void* key, size_t key_length, const void* value, size_t length) {
    void* new_value = std::malloc(length);
    if (new_value == nullptr) {
        return false;
    }
    std::memcpy(new_value, value, length);
    Data* existing = find_data(key, key_length);
    if (existing != nullptr) {
        std::free(existing->p_data);
        existing->p_data = new_value;
        existing->size = static_cast<int>(length);
        return true;
    }
    char* new_key = duplicate_key(static_cast<const char*>(key), key_length);
    Data* node = new_key == nullptr ? nullptr : new (std::nothrow) Data{
        new_key, new_value, static_cast<int>(length), static_cast<uint16_t>(key_length), data_head_};
    if (node == nullptr) {
        std::free(new_key);
        std::free(new_value);
        return false;
    }
    data_head_ = node;
    return true;
}

bool Flash::load_page(size_t index, const PageInfo& info) {
    RecordCursor cursor{kPagePrefixSize, 0U};
    RecordView record{};
    while (next_record(index, info, cursor, record)) {
        if (record.header.type != kSetEntry) {
            continue;
        }
        RecordView latest{};
        if (find_latest(index, info, record.key, record.header.key_length, latest)
            && latest.obsolete_address == record.obsolete_address
            && !store_ram(record.key, record.header.key_length, record.value, record.header.value_length)) {
            return false;
        }
    }
    return true;
}

bool Flash::write_raw(const char* key, const void* value, size_t length) {
    if (!initialized_) {
        init();
    }
    if (!initialized_ || !take_locks()) {
        return false;
    }
    size_t key_size = 0U;
    bool ok = get_key_length(key, &key_size) && value != nullptr && length > 0U && length <= UINT16_MAX;
    if (ok) {
        // Always reserve room for one compacted snapshot of all live values,
        // independent of how much obsolete journal history is present now.
        size_t projected = kBatchOverhead;
        bool found = false;
        for (const Data* item = data_head_; item != nullptr && ok; item = item->next) {
            const bool same_key = item->key_length == key_size && std::memcmp(item->key, key, key_size) == 0;
            const size_t total = entry_size(item->key_length, same_key ? length : static_cast<size_t>(item->size));
            ok = projected <= batch_capacity() && total <= batch_capacity() - projected;
            projected += total;
            found = found || same_key;
        }
        if (ok && !found) {
            ok = projected <= batch_capacity() && entry_size(key_size, length) <= batch_capacity() - projected;
        }
    }
    if (ok) {
        ok = store_ram(key, key_size, value, length);
    }
    give_locks();
    return ok;
}

bool Flash::read_raw(const char* key, void* value) {
    if (!initialized_) {
        init();
    }
    if (!initialized_ || !take_mutex(read_mutex)) {
        return false;
    }
    size_t key_size = 0U;
    bool ok = get_key_length(key, &key_size) && value != nullptr;
    if (ok) {
        const Data* item = find_data(key, key_size);
        ok = item != nullptr;
        if (ok) {
            std::memcpy(value, item->p_data, static_cast<size_t>(item->size));
        }
    }
    give_mutex(read_mutex);
    return ok;
}
