#include "LeptonFLiR.h"
#include "Wire.h"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

#define CHECK(expr) do { \
    if (!(expr)) { \
        std::cerr << "FAIL " << __FILE__ << ':' << __LINE__ << ": " #expr "\n"; \
        ++failures; \
    } \
} while (0)

uint16_t wordAt(const std::vector<uint8_t>& bytes, size_t offset) {
    return static_cast<uint16_t>((static_cast<uint16_t>(bytes[offset]) << 8) | bytes[offset + 1]);
}

const std::vector<uint8_t>* lastWriteTo(uint16_t reg) {
    for (auto it = Wire.transmissions.rbegin(); it != Wire.transmissions.rend(); ++it) {
        if (it->size() >= 4 && wordAt(*it, 0) == reg)
            return &*it;
    }
    return nullptr;
}

uint16_t lastRegisterValue(uint16_t reg) {
    const std::vector<uint8_t>* tx = lastWriteTo(reg);
    CHECK(tx != nullptr);
    return tx ? wordAt(*tx, 2) : 0;
}

std::vector<uint16_t> dataWordsAt(uint16_t reg) {
    const std::vector<uint8_t>* tx = lastWriteTo(reg);
    std::vector<uint16_t> words;
    CHECK(tx != nullptr);
    if (!tx) return words;
    for (size_t i = 2; i + 1 < tx->size(); i += 2)
        words.push_back(wordAt(*tx, i));
    return words;
}

void queueSetOrRun() {
    Wire.queueWords({0, 0});
}

void queueGet(const std::vector<uint16_t>& words) {
    Wire.queueWords({0, 0});
    Wire.queueWords(words);
}

void testAGCCommands() {
    LeptonFLiR camera;

    Wire.clear();
    queueSetOrRun();
    camera.agc_setHEQLinearPercent(37);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 1);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x014D);
    const auto setWords = dataWordsAt(LEP_I2C_DATA_0_REG);
    CHECK(setWords.size() == 1 && setWords[0] == 37);

    Wire.clear();
    queueGet({55});
    CHECK(camera.agc_getHEQLinearPercent() == 55);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 1);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x014C);
}

void testSYSCommands() {
    LeptonFLiR camera;

    Wire.clear();
    queueSetOrRun();
    camera.sys_setGainMode(LEP_SYS_GAIN_MODE_AUTO);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 2);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x0249);
    auto words = dataWordsAt(LEP_I2C_DATA_0_REG);
    CHECK(words.size() == 2 && words[0] == LEP_SYS_GAIN_MODE_AUTO && words[1] == 0);

    LEP_SYS_GAIN_MODE_OBJ object = {};
    object.sysGainModeROI = {1, 2, 158, 118};
    object.sysGainModeThresholds = {10, 11, 12, 13, 14, 15};
    object.sysGainRoiPopulation = 19200;
    object.sysGainModeTempEnabled = 1;
    object.sysGainModeFluxThresholdLowToHigh = 1234;
    object.sysGainModeFluxThresholdHighToLow = 5678;

    static_assert(sizeof(LEP_SYS_GAIN_MODE_OBJ) == 28, "gain mode object must be 14 CCI words");
    Wire.clear();
    queueSetOrRun();
    camera.sys_setGainModeObject(&object);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 14);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x0251);
    words = dataWordsAt(LEP_I2C_DATA_0_REG);
    CHECK(words.size() == 14);
    CHECK(words[0] == 1 && words[3] == 118);
    CHECK(words[10] == 19200 && words[13] == 5678);

    Wire.clear();
    queueGet({LEP_SYS_FFC_DONE, 0});
    CHECK(camera.sys_getFFCState() == LEP_SYS_FFC_DONE);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 2);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x024C);
}

void testVIDCommands() {
    LeptonFLiR camera;

    Wire.clear();
    queueSetOrRun();
    camera.vid_setLowGainPseudoColorLUT(LEP_VID_ICE_FIRE_LUT);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 2);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x0335);
    const auto words = dataWordsAt(LEP_I2C_DATA_0_REG);
    CHECK(words.size() == 2 && words[0] == LEP_VID_ICE_FIRE_LUT && words[1] == 0);

    Wire.clear();
    queueGet({LEP_VID_RAIN_LUT, 0});
    CHECK(camera.vid_getLowGainPseudoColorLUT() == LEP_VID_RAIN_LUT);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x0334);

    LEP_VID_LUT_BUFFER lut;
    std::memset(&lut, 0xA5, sizeof(lut));
    Wire.clear();
    queueGet(std::vector<uint16_t>(sizeof(lut) / 2, 0));
    camera.vid_getUserColorLUT(&lut);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == sizeof(lut) / 2);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x0308);
    CHECK(lut.bin[0].red == 0 && lut.bin[127].green == 0 && lut.bin[255].blue == 0);

    size_t dataBufferSelections = 0;
    for (const auto& tx : Wire.transmissions) {
        if (tx.size() == 2 && wordAt(tx, 0) >= LEP_I2C_DATA_BUFFER)
            ++dataBufferSelections;
    }
    CHECK(dataBufferSelections > 1);
}

void testOEMCommands() {
    LeptonFLiR camera;

    Wire.clear();
    queueSetOrRun();
    camera.oem_setVideoOutputFormat(LEP_OEM_VIDEO_OUTPUT_FORMAT_RAW14);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 2);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x4829);
    auto words = dataWordsAt(LEP_I2C_DATA_0_REG);
    CHECK(words.size() == 2 && words[0] == LEP_OEM_VIDEO_OUTPUT_FORMAT_RAW14 && words[1] == 0);

    std::vector<uint16_t> partWords(16, 0);
    const char part[] = "500-0771-01";
    for (size_t i = 0; i < sizeof(part) - 1; ++i) {
        const size_t w = i / 2;
        if ((i & 1) == 0)
            partWords[w] |= static_cast<uint16_t>(static_cast<uint8_t>(part[i])) << 8;
        else
            partWords[w] |= static_cast<uint8_t>(part[i]);
    }
    Wire.clear();
    queueGet(partWords);
    char partNumber[33] = {};
    camera.oem_getFlirPartNumber(partNumber);
    CHECK(std::string(partNumber) == part);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 16);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x481C);

    Wire.clear();
    queueGet({0x0102, 0x0304, 0x0506, 0xABCD});
    LEP_OEM_SW_VERSION version = {};
    camera.oem_getSoftwareVersion(&version);
    CHECK(version.gpp_major == 1 && version.gpp_minor == 2 && version.gpp_build == 3);
    CHECK(version.dsp_major == 4 && version.dsp_minor == 5 && version.dsp_build == 6);
    CHECK(version.reserved == 0xABCD);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x4820);

    Wire.clear();
    queueSetOrRun();
    camera.oem_setTemporalFilterEnabled(true);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 2);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x4871);
}

void testRADCommands() {
    LeptonFLiR camera;

    static_assert(sizeof(LEP_RBFO) == 16, "RBFO must be 8 CCI words");
    LEP_RBFO rbfo = {};
    rbfo.RBFO_R = 0x11223344u;
    rbfo.RBFO_B = 0x55667788u;
    rbfo.RBFO_F = 0x99AABBCCu;
    rbfo.RBFO_O = static_cast<int32_t>(0xDDEEFF00u);

    Wire.clear();
    queueSetOrRun();
    camera.rad_setRBFOExternalParameters(&rbfo);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 8);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x4E05);
    auto words = dataWordsAt(LEP_I2C_DATA_0_REG);
    CHECK(words.size() == 8);
    CHECK(words[0] == 0x3344 && words[1] == 0x1122);
    CHECK(words[6] == 0xFF00 && words[7] == 0xDDEE);

    Wire.clear();
    queueSetOrRun();
    camera.rad_runFFCNormalization();
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 0);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x4E2E);

    Wire.clear();
    queueGet({LEP_RAD_RESOLUTION_0_01, 0});
    CHECK(camera.rad_getTLinearResolution() == LEP_RAD_RESOLUTION_0_01);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 2);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x4EC4);

    Wire.clear();
    queueGet({30015, 30115, 29915, 4800});
    LEP_RAD_SPOTMETER_VALUES spot = {};
    camera.rad_getSpotmeterValues(&spot);
    CHECK(spot.radSpotmeterValue == 30015);
    CHECK(spot.radSpotmeterMaxValue == 30115);
    CHECK(spot.radSpotmeterMinValue == 29915);
    CHECK(spot.radSpotmeterPopulation == 4800);
    CHECK(lastRegisterValue(LEP_I2C_DATA_LENGTH_REG) == 4);
    CHECK(lastRegisterValue(LEP_I2C_COMMAND_REG) == 0x4ED0);
}

} // namespace

int main() {
    testAGCCommands();
    testSYSCommands();
    testVIDCommands();
    testOEMCommands();
    testRADCommands();

    if (failures) {
        std::cerr << failures << " module test(s) failed\n";
        return 1;
    }

    std::cout << "All LeptonFLiR module tests passed\n";
    return 0;
}
