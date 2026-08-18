#pragma once
#include "Arduino.h"

#include <deque>
#include <vector>

#define BUFFER_LENGTH 32
#define WIRE_INTERFACES_COUNT 1

class TwoWire {
public:
    void begin() {}
    void setClock(uint32_t) {}

    void beginTransmission(uint8_t address) {
        currentAddress = address;
        current.clear();
    }

    uint8_t endTransmission() {
        transmissions.push_back(current);
        current.clear();
        return endTransmissionResult;
    }

    size_t requestFrom(uint8_t, size_t length) {
        return length <= rx.size() ? length : rx.size();
    }

    size_t write(uint8_t value) {
        current.push_back(value);
        return 1;
    }

    int read() {
        if (rx.empty()) return -1;
        const uint8_t value = rx.front();
        rx.pop_front();
        return value;
    }

    void queueWord(uint16_t word) {
        rx.push_back(static_cast<uint8_t>(word >> 8));
        rx.push_back(static_cast<uint8_t>(word & 0xFF));
    }

    void queueWords(const std::vector<uint16_t>& words) {
        for (uint16_t word : words)
            queueWord(word);
    }

    void clear() {
        current.clear();
        rx.clear();
        transmissions.clear();
        endTransmissionResult = 0;
    }

    uint8_t currentAddress = 0;
    uint8_t endTransmissionResult = 0;
    std::vector<uint8_t> current;
    std::deque<uint8_t> rx;
    std::vector<std::vector<uint8_t>> transmissions;
};

extern TwoWire Wire;
extern TwoWire Wire1;
