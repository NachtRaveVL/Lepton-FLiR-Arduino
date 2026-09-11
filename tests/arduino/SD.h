#pragma once
#include "Arduino.h"

#define FILE_WRITE 1

class File {
public:
    explicit operator bool() const { return true; }
    size_t write(const byte*, size_t size) { return size; }
    void close() {}
};

class SDClass {
public:
    bool begin(byte) { return true; }
    bool rmdir(const char*) { return true; }
    bool mkdir(const char*) { return true; }
    bool remove(const char* path) { removed.emplace_back(path); return true; }
    File open(const char* path, int) { opened.emplace_back(path); return File(); }

    std::vector<std::string> removed;
    std::vector<std::string> opened;
};

inline SDClass SD;
