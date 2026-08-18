#pragma once
#include "Arduino.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#ifdef SPI_HAS_TRANSFER_ASYNC
#include "EventResponder.h"
#endif

class SPISettings {
public:
    SPISettings(uint32_t = 0, uint8_t = 0, uint8_t = 0) {}
};

class SPIClass {
public:
    void begin() {}
    void beginTransaction(const SPISettings&) { inTransaction = true; }
    void endTransaction() { inTransaction = false; }

    uint16_t transfer16(uint16_t) {
        if (rx.empty()) return 0;
        const uint16_t value = rx.front();
        rx.pop_front();
        return value;
    }

#ifdef SPI_HAS_TRANSFER_ASYNC
    bool transfer(const void*, void* rxBuffer, size_t count, EventResponderRef event) {
        ++asyncTransferCount;
        if (asyncShouldFail)
            return false;

        auto* out = static_cast<uint8_t*>(rxBuffer);
        for (size_t i = 0; i < count; ++i) {
            const size_t byteInWord = i & 1;
            uint16_t word = 0;
            if (!rx.empty()) word = rx.front();
            if (out)
                out[i] = byteInWord == 0 ? static_cast<uint8_t>(word >> 8) : static_cast<uint8_t>(word & 0xFF);
            if (byteInWord != 0 && !rx.empty())
                rx.pop_front();
        }
        event.triggerEvent();
        return true;
    }
#endif

    void queue(const std::vector<uint16_t>& values) {
        rx.insert(rx.end(), values.begin(), values.end());
    }

    void clear() {
        rx.clear();
        asyncTransferCount = 0;
        asyncShouldFail = false;
    }

    std::deque<uint16_t> rx;
    bool inTransaction = false;
    size_t asyncTransferCount = 0;
    bool asyncShouldFail = false;
};

extern SPIClass SPI;
