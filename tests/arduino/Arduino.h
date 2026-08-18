#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <sstream>
#include <string>

using byte = uint8_t;
using boolean = bool;

#ifndef F_CPU
#define F_CPU 600000000UL
#endif

#define F(x) x
#define SS 10
#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0
#define HEX 16
#define DEC 10
#define MSBFIRST 1
#define SPI_MODE3 3
#define RISING 3

class String {
public:
    String() = default;
    String(const char *value) : value_(value ? value : "") {}
    String(const std::string& value) : value_(value) {}
    String(char value) : value_(1, value) {}
    String(int value) : value_(std::to_string(value)) {}
    String(unsigned int value) : value_(std::to_string(value)) {}
    String(long value) : value_(std::to_string(value)) {}
    String(unsigned long value) : value_(std::to_string(value)) {}

    const char *c_str() const { return value_.c_str(); }
    operator std::string() const { return value_; }

    friend String operator+(const String& lhs, const String& rhs) {
        return String(lhs.value_ + rhs.value_);
    }

private:
    std::string value_;
};

class SerialMock {
public:
    explicit operator bool() const { return true; }
    void begin(unsigned long) {}
    void flush() {}

    template <typename T>
    void print(const T&) {}

    template <typename T>
    void print(const T&, int) {}

    void print(const String&) {}
    void print(const char*) {}

    template <typename T>
    void println(const T&) {}

    template <typename T>
    void println(const T&, int) {}

    void println(const String&) {}
    void println(const char*) {}
    void println() {}
};

extern SerialMock Serial;

inline byte highByte(uint16_t value) { return static_cast<byte>(value >> 8); }
inline byte lowByte(uint16_t value) { return static_cast<byte>(value & 0xFF); }
using std::max;
using std::min;

inline unsigned long millis() { static unsigned long value = 0; return value++; }
inline void delay(unsigned long) {}
inline void yield() {}
inline void pinMode(byte, int) {}
inline void digitalWrite(byte, int) {}
inline int digitalPinToInterrupt(byte pin) { return pin; }
inline void attachInterrupt(int, void (*)(), int) {}

#ifndef PORTC
#define PORTC 0
#endif
#ifndef PORTD
#define PORTD 0
#endif
