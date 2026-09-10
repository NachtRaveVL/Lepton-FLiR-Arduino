#include "LeptonFLiR.h"
#include "SPI.h"
#include "Wire.h"

#include <cstring>
#include <iostream>
#include <vector>

namespace {

int failures = 0;
#define CHECK(expr) do { if (!(expr)) { \
    std::cerr << "\033[31mFAIL\033[0m " << __LINE__ << ": " #expr "\n"; ++failures; \
} } while (0)

class TestLeptonFLiR : public LeptonFLiR {
public:
    using LeptonFLiR::LeptonFLiR;
    using LeptonFLiR::wordsToHexString;
    using LeptonFLiR::receiveCommand;

    void configure(bool segmented = false, LeptonFLiR_TelemetryMode telemetry = LeptonFLiR_TelemetryMode_Header) {
        _cameraType = segmented ? LeptonFLiR_CameraType_Lepton3_5 : LeptonFLiR_CameraType_Lepton2_5;
        _tempMode = LeptonFLiR_TemperatureMode_Celsius;
        if (!_nextFrame) _nextFrame = new FrameSettings(_lastFrame, _frameCounter++);
        _nextFrame->imageMode = segmented ? LeptonFLiR_ImageMode_160x120_16bpp_164Brf : LeptonFLiR_ImageMode_80x60_16bpp_164Brf;
        _nextFrame->outputMode = LeptonFLiR_ImageOutputMode_GS16;
        _nextFrame->telemetryMode = telemetry;
        _nextFrameNeedsUpdate = false;
    }
};

void packet(std::vector<uint16_t>& stream, uint16_t id, const std::vector<uint16_t>& data) {
    stream.push_back(id); stream.push_back(0);
    stream.insert(stream.end(), data.begin(), data.end());
}

void queueGet(std::initializer_list<uint16_t> data) {
    Wire.queueWords({0, 0}); Wire.queueWords(std::vector<uint16_t>(data));
}

void queueRawFrame(uint16_t value) {
    std::vector<uint16_t> stream;
    for (int i = 0; i < 60; ++i) packet(stream, i, std::vector<uint16_t>(80, value));
    SPI.clear(); SPI.queue(stream);
}

void testTelemetry() {
    // Engineering datasheet Rev 203 Tables 2/3. Cover both geometries/locations and all FFC states.
    for (bool segmented : {false, true}) {
        for (auto location : {LeptonFLiR_TelemetryMode_Header, LeptonFLiR_TelemetryMode_Footer}) {
            for (int state = 0; state < 4; ++state) {
                TestLeptonFLiR camera;
                camera.configure(segmented, location);
                std::vector<uint16_t> stream;
                const int count = segmented ? 244 : 63;
                const int firstTelemetry = location == LeptonFLiR_TelemetryMode_Header ? 0 : count - (segmented ? 4 : 3);
                for (int i = 0; i < count; ++i) {
                    std::vector<uint16_t> data(80, 0);
                    if (i == firstTelemetry) {
                        data[0] = 0x0901;
                        data[3] = 0x0010; // shutdown imminent
                        data[4] = 0x1008 | (state << 4); // AGC enabled, FFC desired
                        for (int j = 5; j <= 16; ++j) data[j] = 0xABCD;
                        data[20] = 0x1234; data[21] = 0x5678;
                        data[34] = 2; data[35] = 3; data[36] = 50; data[37] = 70;
                    }
                    int id = segmented ? i % 61 : i;
                    if (segmented && id == 20) id |= (i / 61 + 1) << 12;
                    packet(stream, id, data);
                }
                SPI.clear(); SPI.queue(stream);
                CHECK(camera.tryReadNextFrame());
                auto *t = camera.getTelemetryOutputData();
                CHECK(t != nullptr);
                if (!t) continue;
                CHECK(t->revisionMajor == 9 && t->revisionMinor == 1);
                CHECK(t->ffcDesired && t->agcEnabled && t->shutdownImminent);
                CHECK(camera.getTelemetryAGCEnabled());
                CHECK(camera.getTelemetryShouldRunFFCNormalization() == (state != 2));
                // Existing enum values are preserved; imminent is appended as 3.
                const int expectedStates[] = {0, 3, 1, 2};
                CHECK(static_cast<int>(t->ffcState) == expectedStates[state]);
                CHECK(t->agcRegion.startRow == 2 && t->agcRegion.startCol == 3);
                CHECK(t->agcRegion.endRow == 50 && t->agcRegion.endCol == 70);
                CHECK(t->frameCounter == 0x12345678);
                const void *serialEnd = std::memchr(t->serialNumber, 0, sizeof(t->serialNumber));
                const void *revisionEnd = std::memchr(t->softwareRevision, 0, sizeof(t->softwareRevision));
                CHECK(serialEnd && revisionEnd);
                if (serialEnd) CHECK(std::string(t->serialNumber) == "ABCDABCDABCDABCDABCDABCDABCDABCD");
                if (revisionEnd) CHECK(std::string(t->softwareRevision) == "ABCDABCDABCDABCD");
            }
        }
    }
    // The old masks (bits 2/11) must not cause false positives.
    TestLeptonFLiR camera;
    camera.configure();
    std::vector<uint16_t> stream;
    for (int i = 0; i < 63; ++i) {
        std::vector<uint16_t> data(80, 0);
        if (!i) { data[0] = 0x0901; data[4] = 0x0804; }
        packet(stream, i, data);
    }
    SPI.clear(); SPI.queue(stream);
    CHECK(camera.tryReadNextFrame());
    CHECK(!camera.getTelemetryAGCEnabled());
    CHECK(!camera.getTelemetryShouldRunFFCNormalization());
}

void testHexStrings() {
    const uint16_t words[] = {0x0123, 0xABCD};
    for (int capacity = 0; capacity <= 12; ++capacity) {
        char buffer[14]; std::memset(buffer, '!', sizeof(buffer));
        TestLeptonFLiR::wordsToHexString(words, 2, buffer + 1, capacity);
        CHECK(buffer[0] == '!'); CHECK(buffer[capacity + 1] == '!');
        if (capacity) CHECK(std::memchr(buffer + 1, 0, capacity) != nullptr);
        if (capacity == 9) CHECK(std::string(buffer + 1) == "0123ABCD");
        if (capacity == 10) CHECK(std::string(buffer + 1) == "0123:ABCD");
    }
    char one = '!';
    TestLeptonFLiR::wordsToHexString(nullptr, 2, &one, 1);
    CHECK(one == 0);
}

void queueFrameSettings(bool agc, bool heq, bool wide) {
    queueGet({LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14, 0});
    queueGet({static_cast<uint16_t>(agc), 0});
    if (agc) {
        queueGet({static_cast<uint16_t>(heq ? LEP_AGC_HEQ : LEP_AGC_LINEAR), 0});
        if (heq) queueGet({static_cast<uint16_t>(wide ? LEP_AGC_SCALE_TO_14_BITS : LEP_AGC_SCALE_TO_8_BITS), 0});
    }
    queueGet({0, 0}); // telemetry disabled
}

void testAGCScaleChanges() {
    LeptonFLiR camera;
    camera.init(LeptonFLiR_CameraType_Lepton2);
    // Public setters must invalidate settings between captured frames.
    for (bool wide : {true, false, true}) {
        Wire.clear(); Wire.queueWords({0, 0});
        camera.agc_setHEQScaleFactor(wide ? LEP_AGC_SCALE_TO_14_BITS : LEP_AGC_SCALE_TO_8_BITS);
        queueFrameSettings(true, true, wide);
        queueRawFrame(0x1234);
        CHECK(camera.tryReadNextFrame());
        CHECK(camera.getImageOutputBpp() == (wide ? 2 : 1));
        auto *image = camera.getImageOutputData(); CHECK(image != nullptr);
        if (!image) continue;
        if (wide) {
            uint16_t value; std::memcpy(&value, image, sizeof(value));
            CHECK(value == 0x1234);
            CHECK(camera.getImagePixelData(0, 0).std.value == 0x1234);
        } else CHECK(image[0] == 0x34);
        CHECK(Wire.rx.empty());
    }
    Wire.clear(); Wire.queueWords({0, 0}); camera.agc_setAGCPolicy(LEP_AGC_LINEAR);
    queueFrameSettings(true, false, true); queueRawFrame(0x0056);
    CHECK(camera.tryReadNextFrame()); CHECK(camera.getImageOutputBpp() == 1);
    CHECK(Wire.rx.empty());

    // An unsuccessful scale query must not allow a frame with stale settings.
    Wire.clear(); Wire.queueWords({0, 0}); camera.agc_setAGCPolicy(LEP_AGC_HEQ);
    queueGet({LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14, 0}); queueGet({1, 0}); queueGet({LEP_AGC_HEQ, 0});
    Wire.queueWords({0, 0xFF00}); queueRawFrame(0x1234);
    const size_t queued = SPI.rx.size();
    CHECK(!camera.tryReadNextFrame()); CHECK(SPI.rx.size() == queued);
}

void testRejectedGets() {
    TestLeptonFLiR camera;
    for (bool busy : {false, true}) {
        for (int width : {1, 2, 4}) {
            Wire.clear(); Wire.queueWord(0);
            if (busy) Wire.queueWord(1);
            Wire.queueWord(0xFF00); Wire.queueWords({30000, 31000, 29000, 4800});
            uint16_t one = 111; uint32_t two = 222;
            LEP_RAD_SPOTMETER_VALUES spot = {111, 222, 333, 444};
            if (width == 1) camera.receiveCommand(0x0214, &one);
            else if (width == 2) camera.receiveCommand(0x020C, &two);
            else camera.rad_getSpotmeterValues(&spot);
            CHECK(camera.getLastLepResult() == LEP_ERROR);
            CHECK(one == 111 && two == 222 && spot.radSpotmeterValue == 111);
            CHECK(Wire.rx.size() == 8); // stale data registers were not read
        }
    }
    Wire.clear(); Wire.endTransmissionResult = 2;
    CHECK(camera.sys_getCameraUptime() == 0);
    CHECK(camera.getLastI2CError() == 2);
    Wire.clear();
}

void testVSyncRecovery() {
    TestLeptonFLiR camera(SS, 7);
    camera.init(LeptonFLiR_CameraType_Lepton2_5);
    camera.configure(false, LeptonFLiR_TelemetryMode_Disabled);
    std::vector<uint16_t> stream;
    packet(stream, 0, std::vector<uint16_t>(80)); packet(stream, 5, std::vector<uint16_t>(80));
    SPI.clear(); SPI.queue(stream); arduinoDelays.clear();
    arduinoDelayHook = [](unsigned long ms) {
        if (ms > 185) {
            CHECK(arduinoPinLevels[SS] == HIGH);
            CHECK(SPI.inTransaction); // resync stays within this SPI transaction
            triggerInterrupt(7); // a stale pulse during recovery must be discarded
        }
    };
    triggerInterrupt(7); CHECK(!camera.tryReadNextFrame());
    arduinoDelayHook = nullptr;
    CHECK(!SPI.inTransaction); CHECK(arduinoPinLevels[SS] == HIGH);
    CHECK(std::find(arduinoDelays.begin(), arduinoDelays.end(), 186) != arduinoDelays.end());
    queueRawFrame(0x1357); const size_t queued = SPI.rx.size();
    CHECK(!camera.tryReadNextFrame()); CHECK(SPI.rx.size() == queued);
    arduinoDelays.clear(); triggerInterrupt(7); CHECK(camera.tryReadNextFrame());
    CHECK(camera.getImagePixelData(0, 0).std.value == 0x1357);
    CHECK(arduinoDelays.empty()); // healthy capture does not hard-resync
}

} // namespace

int main() {
    testTelemetry(); testHexStrings(); testAGCScaleChanges(); testRejectedGets(); testVSyncRecovery();
    if (failures) { std::cerr << failures << " regression assertions failed\n"; return 1; }
    std::cout << "\033[32mPASS\033[0m LeptonFLiR specification regressions\n";
}
