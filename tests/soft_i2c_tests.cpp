#include "LeptonFLiR.h"

#include <deque>
#include <iostream>
#include <vector>

namespace {
int failures = 0;
#define CHECK(expr) do { if (!(expr)) { \
    std::cerr << "\033[31mFAIL\033[0m " << __LINE__ << ": " #expr "\n"; ++failures; \
} } while (0)

std::vector<uint8_t> starts, writes;
std::vector<bool> lastReads;
std::deque<uint8_t> reads;
int stops = 0, nackStart = -1, nackWrite = -1;
void reset() {
    starts.clear(); writes.clear(); lastReads.clear(); reads.clear();
    stops = 0; nackStart = nackWrite = -1;
}
class TestLeptonFLiR : public LeptonFLiR {
public:
    using LeptonFLiR::writeRegister;
    using LeptonFLiR::readRegister;
    using LeptonFLiR::readDataRegister;
    using LeptonFLiR::i2cWire_requestFrom;
    using LeptonFLiR::i2cWire_read;
};
} // namespace

// Link the real software-I2C library path against observable low-level bus operations.
void LEPFLIR_i2c_stop(void) asm("ass_i2c_stop");
bool LEPFLIR_i2c_write(uint8_t value) asm("ass_i2c_write");
boolean i2c_init(void) { return true; }
bool i2c_start(uint8_t address) {
    starts.push_back(address);
    return static_cast<int>(starts.size()) - 1 != nackStart;
}
void LEPFLIR_i2c_stop(void) { ++stops; }
bool LEPFLIR_i2c_write(uint8_t value) {
    writes.push_back(value);
    return static_cast<int>(writes.size()) - 1 != nackWrite;
}
uint8_t i2c_read(bool last) {
    lastReads.push_back(last);
    if (reads.empty()) { CHECK(false); return 0; }
    auto value = reads.front(); reads.pop_front(); return value;
}

int main() {
    TestLeptonFLiR camera;
    reset();
    CHECK(camera.writeRegister(0x0004, 0x1234) == 0);
    CHECK(starts == std::vector<uint8_t>({0x54}));
    CHECK(writes == std::vector<uint8_t>({0x00, 0x04, 0x12, 0x34}));
    CHECK(stops == 1);

    reset(); reads = {0x12, 0x34}; uint16_t result = 0;
    CHECK(camera.readRegister(0x0002, &result) == 0);
    CHECK(result == 0x1234);
    CHECK(starts == std::vector<uint8_t>({0x54, 0x55}));
    CHECK(lastReads == std::vector<bool>({false, true}));
    CHECK(stops == 2); // write-address STOP, read completion STOP

    reset(); nackStart = 0;
    CHECK(camera.writeRegister(0x0004, 0x1234) == 2);
    CHECK(camera.getLastI2CError() == 2);
    CHECK(writes.empty() && stops == 1);

    reset(); nackWrite = 2;
    CHECK(camera.writeRegister(0x0004, 0x1234) == 3);
    CHECK(writes.size() == 3 && stops == 1);

    reset(); nackStart = 1; result = 0xBEEF;
    CHECK(camera.readRegister(0x0002, &result) == 2);
    CHECK(result == 0xBEEF && lastReads.empty() && stops == 2);

    reset(); nackStart = 1; uint16_t data[20] = {};
    CHECK(camera.readDataRegister(data, 20) == 2);
    CHECK(lastReads.empty() && stops == 2);

    reset();
    for (int i = 0; i < 40; ++i) reads.push_back(static_cast<uint8_t>(i));
    CHECK(camera.readDataRegister(data, 20) == 0);
    CHECK(data[0] == 0x0001 && data[19] == 0x2627);
    CHECK(starts == std::vector<uint8_t>({0x54, 0x55, 0x54, 0x55}));
    CHECK(writes == std::vector<uint8_t>({0xF8, 0x00, 0xF8, 0x20}));
    CHECK(stops == 4 && lastReads.size() == 40);
    for (size_t i = 0; i < lastReads.size(); ++i)
        CHECK(lastReads[i] == (i == 31 || i == 39));

    reset(); reads = {0xAB};
    CHECK(camera.i2cWire_requestFrom(0x2A, 1) == 1);
    CHECK(camera.i2cWire_read() == 0xAB);
    CHECK(lastReads == std::vector<bool>({true}) && stops == 1);
    CHECK(camera.i2cWire_read() == 0); // exhausted reads must not clock the bus
    CHECK(lastReads.size() == 1 && stops == 1);
    CHECK(camera.i2cWire_requestFrom(0x2A, 0) == 0);
    CHECK(starts.size() == 1);

    if (failures) { std::cerr << failures << " software-I2C assertions failed\n"; return 1; }
    std::cout << "\033[32mPASS\033[0m LeptonFLiR software-I2C transactions\n";
}
