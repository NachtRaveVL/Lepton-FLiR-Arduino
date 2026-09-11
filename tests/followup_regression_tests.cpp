#include "LeptonFLiR.h"
#include "SD.h"

#include <iostream>
#include <limits>
#include <vector>

void writeBMPFile(File& bmpFile, byte* imageData, int width, int height, int pitch);
#include "../examples/ImageCaptureExample/ImageCaptureExample.ino"

namespace {

int failures = 0;
#define CHECK(expr) do { if (!(expr)) { \
    std::cerr << "\033[31mFAIL\033[0m " << __LINE__ << ": " #expr "\n"; ++failures; \
} } while (0)

class TestLeptonFLiR : public LeptonFLiR {
public:
    using LeptonFLiR::waitCommandBegin;
    using LeptonFLiR::waitCommandFinish;
};

void queueGet(std::initializer_list<uint16_t> words) {
    Wire.queueWords({0, 0});
    Wire.queueWords(std::vector<uint16_t>(words));
}

void queueSettings(bool agc, bool radiometric, bool telemetry) {
    queueGet({LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14, 0});
    queueGet({static_cast<uint16_t>(agc), 0});
    if (agc && radiometric) queueGet({LEP_AGC_LINEAR, 0});
    if (radiometric) queueGet({0, 0}); // TLinear disabled
    queueGet({static_cast<uint16_t>(telemetry), 0});
    if (telemetry) queueGet({LEP_TELEMETRY_LOCATION_HEADER, 0});
}

void queueFrame(uint16_t pixel, bool telemetry = false, uint32_t frameNumber = 0,
                const uint16_t* thresholds = nullptr) {
    std::vector<uint16_t> stream;
    for (int line = 0; line < (telemetry ? 63 : 60); ++line) {
        stream.push_back(static_cast<uint16_t>(line));
        stream.push_back(0);
        std::vector<uint16_t> data(80, pixel);
        if (telemetry && line < 3) {
            std::fill(data.begin(), data.end(), 0);
            if (line == 0) {
                data[0] = 0x0901;
                data[20] = static_cast<uint16_t>(frameNumber >> 16);
                data[21] = static_cast<uint16_t>(frameNumber);
            }
            if (line == 2 && thresholds)
                std::copy(thresholds, thresholds + 4, data.begin() + 8);
        }
        stream.insert(stream.end(), data.begin(), data.end());
    }
    SPI.clear(); SPI.queue(stream);
}

void testImageCaptureExample() {
    // Run the actual sketch loop. Datasheet Figure 25 shows counters 0,0,0,3,3,3,6...
    Wire.clear(); SD.opened.clear(); SD.removed.clear();
    flirController.init(LeptonFLiR_CameraType_Lepton1);
    queueSettings(true, false, true);
    hasLastFrameNumber = false;
    for (uint32_t number : {0u, 0u, 3u, 3u, 6u}) {
        queueFrame(0x34, true, number);
        loop();
    }
    const std::vector<std::string> expected = {"FLIR/IMG0000.BMP", "FLIR/IMG0001.BMP", "FLIR/IMG0002.BMP"};
    CHECK(SD.opened == expected);
    CHECK(SD.removed == expected);
    CHECK(Wire.rx.empty());
}

void testResetRefresh() {
    for (bool reboot : {false, true}) {
        LeptonFLiR camera;
        camera.init(LeptonFLiR_CameraType_Lepton2_5);
        Wire.clear(); queueSettings(true, true, false); queueFrame(0x34);
        CHECK(camera.tryReadNextFrame());
        CHECK(camera.getImageOutputBpp() == 1);
        CHECK(Wire.rx.empty());

        Wire.queueWords({0, 0});
        if (reboot) camera.oem_runReboot();
        else camera.oem_runUserDefaultsRestore();
        CHECK(camera.getLastLepResult() == LEP_OK);
        CHECK(Wire.rx.empty());
        CHECK(camera.getImageOutputBpp() == 1); // The last captured frame retains its format.

        // Once the camera is ready, its restored settings have AGC disabled.
        queueSettings(false, true, false); queueFrame(0x1234);
        CHECK(camera.tryReadNextFrame());
        CHECK(camera.getImageOutputBpp() == 2);
        CHECK(camera.getImagePixelData(0, 0).std.value == 0x1234);
        CHECK(Wire.rx.empty());
    }
}

float expectedTemperature(float celsius, LeptonFLiR_TemperatureMode mode) {
    if (mode == LeptonFLiR_TemperatureMode_Fahrenheit) return celsius * 1.8f + 32.0f;
    if (mode == LeptonFLiR_TemperatureMode_Kelvin) return celsius + 273.15f;
    return celsius;
}

void testTemperatureRange() {
    // IDD Rev 303 section 4.5.21: Celsius thresholds 0..600, Kelvin thresholds 0..900.
    const uint16_t cases[][4] = {{0, 0, 0, 0}, {115, 85, 388, 358},
                                {383, 382, 656, 655}, {600, 400, 900, 700}};
    for (auto mode : {LeptonFLiR_TemperatureMode_Celsius, LeptonFLiR_TemperatureMode_Fahrenheit,
                      LeptonFLiR_TemperatureMode_Kelvin}) {
        LeptonFLiR camera;
        camera.init(LeptonFLiR_CameraType_Lepton2_5, mode);
        Wire.clear(); queueSettings(false, true, true);
        for (const auto& values : cases) {
            queueFrame(0, true, 0, values);
            CHECK(camera.tryReadNextFrame());
            const auto* t = camera.getTelemetryOutputData();
            CHECK(t != nullptr);
            if (!t) continue;
            CHECK(std::abs(t->radGainModeSwitchHtLTemp - expectedTemperature(values[0], mode)) < 0.02f);
            CHECK(std::abs(t->radGainModeSwitchLtHTemp - expectedTemperature(values[1], mode)) < 0.02f);
            CHECK(std::abs(t->tlinearGainModeSwitchHtLTemp - expectedTemperature(values[2] - 273.15f, mode)) < 0.02f);
            CHECK(std::abs(t->tlinearGainModeSwitchLtHTemp - expectedTemperature(values[3] - 273.15f, mode)) < 0.02f);
        }
        CHECK(Wire.rx.empty());
        // README's 0.1 K example, including a high scene temperature and full pixel range.
        for (uint16_t pixel : {uint16_t(3000), uint16_t(8731), uint16_t(65535)}) {
            uint32_t kelvin100 = pixel;
            kelvin100 *= 10;
            CHECK(std::abs(camera.kelvin100ToTemperature(kelvin100) -
                           expectedTemperature(pixel * 0.1f - 273.15f, mode)) < 0.02f);
        }
    }
}

void testCommandRollover() {
    TestLeptonFLiR camera;
    // Match millis()'s unsigned-long width, including on 64-bit host builds.
    const unsigned long wrapStart = std::numeric_limits<unsigned long>::max() - 4;
    for (bool finish : {false, true}) {
        for (unsigned long start : {0UL, wrapStart}) {
            for (int timeout : {100, 0, -1}) {
                arduinoMillis = start; Wire.clear();
                Wire.queueWords({1, 1, 1, 1, 1, 1, 1, 0});
                CHECK(finish ? camera.waitCommandFinish(timeout) : camera.waitCommandBegin(timeout));
                CHECK(camera.getLastLepResult() == LEP_OK);
                CHECK(Wire.rx.empty());
            }
            arduinoMillis = start; Wire.clear();
            Wire.queueWords(std::vector<uint16_t>(64, 1));
            CHECK(!(finish ? camera.waitCommandFinish(10) : camera.waitCommandBegin(10)));
            CHECK(camera.getLastLepResult() == LEP_TIMEOUT_ERROR);
            const unsigned long elapsed = arduinoMillis - start;
            CHECK(elapsed >= 10 && elapsed <= 12);
        }
    }
    arduinoMillis = 0; Wire.clear();
}

} // namespace

int main() {
    testImageCaptureExample(); testResetRefresh(); testTemperatureRange(); testCommandRollover();
    if (failures) { std::cerr << failures << " regression assertions failed\n"; return 1; }
    std::cout << "\033[32mPASS\033[0m LeptonFLiR follow-up regressions\n";
}
