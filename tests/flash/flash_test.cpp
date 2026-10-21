#include "flash.hpp"
#include "stm32g4xx_hal.h"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/mman.h>
#include <vector>

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); std::abort(); \
} } while (false)

namespace {
constexpr uintptr_t kStorage = FLASH_BASE + FLASH_SIZE - 2U * FLASH_PAGE_SIZE;
struct Operation { bool erase; uintptr_t address; };
std::array<bool, FLASH_SIZE / 8U> programmed{};
std::vector<Operation> operations;
int program_count = 0;
int erase_count = 0;
int duplicate_attempts = 0;
int cut_at = -1;
int cut_mode = 0; // 0: before, 1: first half, 2: complete, 3: ECC consumed but no visible bits.
bool halted = false;
bool locked = true;
std::array<FakeMutex, 2> mutexes{};
size_t mutex_count = 0U;

struct Image {
    std::array<uint8_t, 2U * FLASH_PAGE_SIZE> bytes;
    std::array<bool, FLASH_SIZE / 8U> touched;
};

void reboot() {
    halted = false;
    locked = true;
    cut_at = -1;
    operations.clear();
    program_count = erase_count = duplicate_attempts = 0;
    for (auto& mutex : mutexes) {
        CHECK(!mutex.held);
    }
}

void blank() {
    reboot();
    std::memset(reinterpret_cast<void*>(FLASH_BASE), 0xFF, FLASH_SIZE);
    programmed.fill(false);
}

Image image() {
    Image saved{};
    std::memcpy(saved.bytes.data(), reinterpret_cast<const void*>(kStorage), saved.bytes.size());
    saved.touched = programmed;
    return saved;
}

void restore(const Image& saved) {
    reboot();
    std::memcpy(reinterpret_cast<void*>(kStorage), saved.bytes.data(), saved.bytes.size());
    programmed = saved.touched;
}

void expect(Flash& flash, const char* key, uint32_t expected) {
    uint32_t actual = UINT32_MAX;
    CHECK(flash.read_value(key, actual));
    CHECK(actual == expected);
}

void absent(Flash& flash, const char* key) {
    uint32_t actual = 0xAABBCCDDU;
    CHECK(!flash.read_value(key, actual));
    CHECK(actual == 0xAABBCCDDU);
}

void append_behavior() {
    blank();
    Flash flash;
    flash.init();
    CHECK(flash.fresh());
    CHECK(operations.empty());
    CHECK(flash.write_value("a", uint32_t{1}));
    CHECK(flash.write_value("b", uint32_t{2}));
    CHECK(operations.empty());
    CHECK(flash.fresh());
    CHECK(erase_count == 1);
    const Image initial = image();
    const int writes = program_count;
    for (int i = 0; i < 10; ++i) {
        CHECK(flash.write_value("a", uint32_t{1}));
        CHECK(flash.fresh());
    }
    CHECK(program_count == writes);
    CHECK(flash.write_value("a", uint32_t{3}));
    CHECK(flash.write_value("a", uint32_t{1}));
    CHECK(flash.write_value("temporary", uint32_t{99}));
    flash.remove("temporary");
    CHECK(flash.fresh());
    CHECK(program_count == writes);
    CHECK(flash.write_value("a", uint32_t{3}));
    const size_t append_start = operations.size();
    CHECK(flash.fresh());
    CHECK(erase_count == 1);
    CHECK(program_count > writes);
    // Retirement programs a previously untouched word in the old batch, after
    // programming the new batch at the tail. Unchanged key b is not rewritten.
    CHECK(operations.back().address < operations[append_start].address);
    CHECK(!initial.touched[(operations.back().address - FLASH_BASE) / 8U]);
    const auto* marker = reinterpret_cast<const uint32_t*>(operations.back().address);
    CHECK(marker[0] == 0x32444C4FU && marker[1] == ~marker[0]);
    flash.init();
    expect(flash, "a", 3);
    expect(flash, "b", 2);
    CHECK(flash.fresh());
    flash.remove("a");
    CHECK(flash.fresh());
    flash.init();
    absent(flash, "a");
    expect(flash, "b", 2);
    CHECK(flash.write_value("a", uint32_t{4}));
    CHECK(flash.fresh());
    flash.init();
    expect(flash, "a", 4);
    uint16_t wrong_size = 123;
    CHECK(!flash.read_value("a", wrong_size));
    CHECK(wrong_size == 123);
    CHECK(!flash.write_raw("", &wrong_size, sizeof(wrong_size)));
    CHECK(!flash.write_raw("bad", nullptr, 1));
    CHECK(!flash.write_raw("bad", &wrong_size, 0));
    CHECK(duplicate_attempts == 0);
}

void capacity_and_collection() {
    blank();
    Flash flash;
    flash.init();
    std::array<uint8_t, 1984> value{};
    // The largest live snapshot exactly fills one 2 KiB page.
    CHECK(flash.write_raw("x", value.data(), 1983));
    CHECK(!flash.write_raw("x", value.data(), 1984));
    CHECK(!flash.write_value("another", uint32_t{1}));
    CHECK(flash.fresh());
    for (int i = 1; i <= 6; ++i) {
        value[0] = static_cast<uint8_t>(i);
        CHECK(flash.write_raw("x", value.data(), 1983));
        CHECK(flash.fresh());
        flash.init();
        std::array<uint8_t, 1983> actual{};
        CHECK(flash.read_raw("x", actual.data(), actual.size()));
        CHECK(std::memcmp(actual.data(), value.data(), actual.size()) == 0);
    }
    CHECK(erase_count == 7);
    CHECK(duplicate_attempts == 0);
    flash.remove("x");
    CHECK(flash.fresh());
    flash.init();
    absent(flash, "x");
    CHECK(flash.fresh());
    CHECK(flash.write_value("new", uint32_t{5}));
    CHECK(flash.fresh());
    flash.init();
    expect(flash, "new", 5);
}

void format_reset() {
    for (int kind = 0; kind < 3; ++kind) {
        blank();
        {
            Flash flash;
            flash.init();
            CHECK(flash.write_value("a", uint32_t{1}));
            CHECK(flash.fresh());
        }
        auto* storage = reinterpret_cast<uint8_t*>(kStorage);
        if (kind == 0) {
            // Previous project's old snapshot magic, deliberately unsupported.
            const uint32_t old_magic = 0x50464B56U;
            std::memcpy(storage, &old_magic, sizeof(old_magic));
        } else if (kind == 1) {
            storage[4] ^= 0x01U; // incompatible version
        } else {
            storage[24U + 16U + 8U] ^= 0x01U; // committed record contents
        }
        reboot();
        Flash flash;
        flash.init();
        CHECK(erase_count == 2);
        absent(flash, "a");
        CHECK(flash.write_value("after_reset", uint32_t{9}));
        CHECK(flash.fresh());
        flash.init();
        expect(flash, "after_reset", 9);
    }
}

void change_batch(Flash& flash) {
    CHECK(flash.write_value("a", uint32_t{11}));
    CHECK(flash.write_value("b", uint32_t{22}));
    CHECK(flash.write_value("c", uint32_t{33}));
    flash.remove("removed");
}

void expect_batch(Flash& flash, bool changed) {
    expect(flash, "a", changed ? 11 : 1);
    expect(flash, "b", changed ? 22 : 2);
    if (changed) {
        expect(flash, "c", 33);
        absent(flash, "removed");
    } else {
        absent(flash, "c");
        expect(flash, "removed", 3);
    }
}

void power_cut_append() {
    blank();
    {
        Flash flash;
        flash.init();
        CHECK(flash.write_value("a", uint32_t{1}));
        CHECK(flash.write_value("b", uint32_t{2}));
        CHECK(flash.write_value("removed", uint32_t{3}));
        CHECK(flash.fresh());
    }
    const Image baseline = image();
    reboot();
    {
        Flash flash;
        flash.init();
        change_batch(flash);
        CHECK(flash.fresh());
    }
    const auto normal_operations = operations;
    CHECK(erase_count == 0);
    int cases = 0;
    for (size_t cut = 0; cut < normal_operations.size(); ++cut) {
        for (int mode = 0; mode < 4; ++mode) {
            restore(baseline);
            {
                Flash flash;
                flash.init();
                change_batch(flash);
                cut_at = static_cast<int>(cut);
                cut_mode = mode;
                (void)flash.fresh();
                CHECK(halted);
                CHECK(duplicate_attempts == 0);
            }
            reboot();
            {
                Flash recovered;
                recovered.init();
                CHECK(erase_count == 0); // torn appends must not clear valid history
                uint32_t a = 0;
                CHECK(recovered.read_value("a", a));
                CHECK(a == 1 || a == 11);
                expect_batch(recovered, a == 11);
                change_batch(recovered);
                // An invisible consumed ECC word can require one failed append
                // before the next call switches to a freshly erased page.
                if (!recovered.fresh()) {
                    CHECK(recovered.fresh());
                }
                recovered.init();
                expect_batch(recovered, true);
            }
            ++cases;
        }
    }
    std::printf("append fault cases: %d\n", cases);
}

void power_cut_collection() {
    blank();
    std::array<uint8_t, 500> payload{};
    {
        Flash flash;
        flash.init();
        CHECK(flash.write_value("b", uint32_t{2}));
        for (int i = 0; i < 6; ++i) {
            payload.fill(static_cast<uint8_t>(i));
            CHECK(flash.write_raw("large", payload.data(), payload.size()));
            CHECK(flash.fresh());
        }
    }
    const Image baseline = image();
    reboot();
    payload.fill(6U);
    {
        Flash flash;
        flash.init();
        CHECK(flash.write_raw("large", payload.data(), payload.size()));
        CHECK(flash.write_value("b", uint32_t{22}));
        CHECK(flash.fresh());
        CHECK(erase_count == 1);
    }
    const auto normal_operations = operations;
    int cases = 0;
    for (size_t cut = 0; cut < normal_operations.size(); ++cut) {
        for (int mode = 0; mode < 4; ++mode) {
            restore(baseline);
            {
                Flash flash;
                flash.init();
                CHECK(flash.write_raw("large", payload.data(), payload.size()));
                CHECK(flash.write_value("b", uint32_t{22}));
                cut_at = static_cast<int>(cut);
                cut_mode = mode;
                (void)flash.fresh();
                CHECK(halted);
                CHECK(duplicate_attempts == 0);
            }
            reboot();
            {
                Flash recovered;
                recovered.init();
                CHECK(erase_count == 0);
                std::array<uint8_t, 500> actual{};
                CHECK(recovered.read_raw("large", actual.data(), actual.size()));
                CHECK(actual[0] == 5U || actual[0] == 6U);
                for (uint8_t byte : actual) {
                    CHECK(byte == actual[0]);
                }
                expect(recovered, "b", actual[0] == 5U ? 2 : 22);
                CHECK(recovered.write_raw("large", payload.data(), payload.size()));
                CHECK(recovered.write_value("b", uint32_t{22}));
                CHECK(recovered.fresh());
                recovered.init();
                expect(recovered, "b", 22);
                CHECK(recovered.read_raw("large", actual.data(), actual.size()));
                CHECK(actual == payload);
                CHECK(duplicate_attempts == 0);
            }
            ++cases;
        }
    }
    std::printf("collection fault cases: %d\n", cases);
}

void clear_failure() {
    blank();
    Flash flash;
    flash.init();
    CHECK(flash.write_value("a", uint32_t{1}));
    CHECK(flash.fresh());
    operations.clear();
    cut_at = 0;
    cut_mode = 1;
    CHECK(!flash.clear());
    expect(flash, "a", 1); // failure retains RAM
    reboot();
    CHECK(flash.clear());
    CHECK(erase_count == 2);
    absent(flash, "a");
}

void random_changes() {
    blank();
    struct Value { bool present{}; uint8_t size{}; std::array<uint8_t, 80> bytes{}; };
    std::array<Value, 8> ram{};
    auto saved = ram;
    uint32_t random = 123456789U;
    auto next = [&random]() {
        random = random * 1664525U + 1013904223U;
        return random;
    };
    Flash flash;
    flash.init();
    for (int step = 0; step < 2500; ++step) {
        const size_t index = (next() >> 16U) % ram.size();
        char key[8];
        std::snprintf(key, sizeof(key), "k%zu", index);
        auto& value = ram[index];
        if ((next() >> 16U) % 5U == 0U) {
            value.present = false;
            flash.remove(key);
        } else {
            value.present = true;
            value.size = static_cast<uint8_t>(1U + (next() >> 16U) % value.bytes.size());
            for (size_t i = 0; i < value.size; ++i) {
                value.bytes[i] = static_cast<uint8_t>(next() >> 16U);
            }
            CHECK(flash.write_raw(key, value.bytes.data(), value.size));
        }
        if ((next() >> 16U) % 4U == 0U) {
            CHECK(flash.fresh());
            saved = ram;
        }
        if ((next() >> 16U) % 13U == 0U) {
            flash.init();
            ram = saved;
        }
        for (size_t i = 0; i < ram.size(); ++i) {
            std::snprintf(key, sizeof(key), "k%zu", i);
            std::array<uint8_t, 80> actual{};
            const bool found = flash.read_raw(key, actual.data(), ram[i].present ? ram[i].size : 1U);
            CHECK(found == ram[i].present);
            if (found) {
                CHECK(std::memcmp(actual.data(), ram[i].bytes.data(), ram[i].size) == 0);
            }
        }
    }
    CHECK(erase_count > 0);
    CHECK(duplicate_attempts == 0);
    std::printf("random operations: 2500, page erases: %d\n", erase_count);
}
} // namespace

SemaphoreHandle_t xSemaphoreCreateMutex() {
    CHECK(mutex_count < mutexes.size());
    return &mutexes[mutex_count++];
}
BaseType_t xSemaphoreTake(SemaphoreHandle_t mutex, uint32_t) {
    CHECK(mutex != nullptr && !mutex->held);
    mutex->held = true;
    return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t mutex) {
    CHECK(mutex != nullptr && mutex->held);
    mutex->held = false;
    return pdTRUE;
}
HAL_StatusTypeDef HAL_FLASH_Unlock() {
    locked = false;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Lock() {
    locked = true;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t, uint32_t address, uint64_t word) {
    CHECK(!locked);
    CHECK(address >= kStorage && address + 8U <= FLASH_BASE + FLASH_SIZE && (address & 7U) == 0U);
    if (halted) {
        return HAL_ERROR;
    }
    if (programmed[(address - FLASH_BASE) / 8U]) {
        ++duplicate_attempts;
        return HAL_ERROR;
    }
    operations.push_back({false, address});
    ++program_count;
    const bool cut = static_cast<int>(operations.size()) - 1 == cut_at;
    if (!cut || cut_mode != 0) {
        const size_t length = cut && cut_mode == 1 ? 4U : 8U;
        programmed[(address - FLASH_BASE) / 8U] = true;
        if (!cut || cut_mode != 3) {
            std::memcpy(reinterpret_cast<void*>(static_cast<uintptr_t>(address)), &word, length);
        }
    }
    if (cut) {
        halted = true;
        return HAL_ERROR;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef* erase, uint32_t*) {
    CHECK(!locked);
    CHECK(erase->Banks == FLASH_BANK_1 && erase->NbPages == 1U);
    const uintptr_t address = FLASH_BASE + erase->Page * FLASH_PAGE_SIZE;
    CHECK(address == kStorage || address == kStorage + FLASH_PAGE_SIZE);
    if (halted) {
        return HAL_ERROR;
    }
    operations.push_back({true, address});
    ++erase_count;
    const bool cut = static_cast<int>(operations.size()) - 1 == cut_at;
    if (!cut || cut_mode != 0) {
        const size_t length = cut && cut_mode == 1 ? FLASH_PAGE_SIZE / 2U : FLASH_PAGE_SIZE;
        std::memset(reinterpret_cast<void*>(address), 0xFF, length);
        for (size_t i = 0; i < length / 8U; ++i) {
            programmed[(address - FLASH_BASE) / 8U + i] = false;
        }
    }
    if (cut) {
        halted = true;
        return HAL_ERROR;
    }
    return HAL_OK;
}

void generated_image(const char* filename) {
    blank();
    FILE* file = std::fopen(filename, "rb");
    CHECK(file != nullptr);
    CHECK(std::fread(reinterpret_cast<void*>(kStorage), 1, 4096, file) == 4096);
    CHECK(std::fgetc(file) == EOF);
    CHECK(std::fclose(file) == 0);
    for (size_t offset = 0; offset < 4096; offset += 8) {
        uint64_t word;
        std::memcpy(&word, reinterpret_cast<void*>(kStorage + offset), 8);
        programmed[(kStorage - FLASH_BASE + offset) / 8] = word != UINT64_MAX;
    }
    Flash flash;
    flash.init();
    CHECK(erase_count == 0);
    expect(flash, "test/u32", UINT32_MAX);
    int32_t i32 = 0;
    int64_t i64 = 0;
    uint64_t u64 = 0;
    float f32 = 0;
    double f64 = 0;
    CHECK(flash.read_value("test/i32", i32) && i32 == INT32_MIN);
    CHECK(flash.read_value("test/i64", i64) && i64 == INT64_MIN);
    CHECK(flash.read_value("test/u64", u64) && u64 == UINT64_MAX);
    CHECK(flash.read_value("test/f32", f32) && f32 == 1.5f);
    CHECK(flash.read_value("test/f64", f64) && f64 == -2.5);
    CHECK(flash.write_value("test/u32", uint32_t{42}));
    CHECK(flash.fresh());
    CHECK(erase_count == 0 && duplicate_attempts == 0);
    flash.init();
    expect(flash, "test/u32", 42);
}

int main(int argc, char** argv) {
    void* mapping = mmap(reinterpret_cast<void*>(FLASH_BASE), FLASH_SIZE, PROT_READ | PROT_WRITE,
                         MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    CHECK(mapping == reinterpret_cast<void*>(FLASH_BASE));
    if (argc == 2) { generated_image(argv[1]); }
    append_behavior();
    capacity_and_collection();
    format_reset();
    power_cut_append();
    power_cut_collection();
    clear_failure();
    random_changes();
    std::puts("Flash journal tests passed");
    CHECK(munmap(mapping, FLASH_SIZE) == 0);
}
