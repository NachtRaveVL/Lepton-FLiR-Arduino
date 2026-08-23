#include "LeptonFLiR.h"
#include "SPI.h"

#include <cstdint>
#include <iostream>
#include <vector>

namespace {

int failures = 0;
#define CHECK(expr) do { \
    if (!(expr)) { \
        std::cerr << "FAIL " << __FILE__ << ':' << __LINE__ << ": " #expr "\n"; \
        ++failures; \
    } \
} while (0)

class TestLeptonFLiR : public LeptonFLiR {
public:
    using LeptonFLiR::LeptonFLiR;

    void configureNext(LeptonFLiR_CameraType cameraType,
                       LeptonFLiR_ImageMode imageMode,
                       LeptonFLiR_ImageOutputMode outputMode) {
        _cameraType = cameraType;
        _tempMode = LeptonFLiR_TemperatureMode_Celsius;
        if (!_nextFrame)
            _nextFrame = new FrameSettings(_lastFrame, _frameCounter++);
        _nextFrame->imageMode = imageMode;
        _nextFrame->outputMode = outputMode;
        _nextFrame->telemetryMode = LeptonFLiR_TelemetryMode_Disabled;
        _nextFrame->agcEnabled = false;
        _nextFrame->tlinearEnabled = false;
        _nextFrame->pclutEnabled = false;
        _nextFrameNeedsUpdate = false;
    }
};

std::vector<uint16_t> rawPayload(uint16_t base) {
    std::vector<uint16_t> payload(80);
    for (int i = 0; i < 80; ++i)
        payload[i] = static_cast<uint16_t>(base + i);
    return payload;
}

void appendPacket(std::vector<uint16_t>& stream, uint16_t id, const std::vector<uint16_t>& payload) {
    stream.push_back(id);
    stream.push_back(0);
    stream.insert(stream.end(), payload.begin(), payload.end());
}

void queue80RawFrame(uint16_t base) {
    std::vector<uint16_t> stream;
    for (int row = 0; row < 60; ++row)
        appendPacket(stream, static_cast<uint16_t>(row), rawPayload(static_cast<uint16_t>(base + row * 80)));
    SPI.clear();
    SPI.queue(stream);
}

void testVSyncFrameReady() {
    {
        TestLeptonFLiR camera(SS, 7);
        camera.init(LeptonFLiR_CameraType_Lepton2);
        camera.configureNext(LeptonFLiR_CameraType_Lepton2,
                             LeptonFLiR_ImageMode_80x60_16bpp_164Brf,
                             LeptonFLiR_ImageOutputMode_GS16);

        CHECK(arduinoInterruptNumber == 7);
        CHECK(arduinoInterruptMode == RISING);
        CHECK(arduinoInterruptHandler != nullptr);

        queue80RawFrame(0x1000);
        const size_t queuedWords = SPI.rx.size();
        CHECK(!camera.tryReadNextFrame());
        CHECK(SPI.rx.size() == queuedWords);

        triggerInterrupt(7);
        CHECK(camera.tryReadNextFrame());
        CHECK(camera.getImagePixelData(3, 5).std.value == static_cast<uint16_t>(0x1000 + 3 * 80 + 5));

        queue80RawFrame(0x2000);
        CHECK(!camera.tryReadNextFrame());
        triggerInterrupt(7);
        CHECK(camera.tryReadNextFrame());
        CHECK(camera.getImagePixelData(4, 6).std.value == static_cast<uint16_t>(0x2000 + 4 * 80 + 6));
    }

    CHECK(arduinoInterruptNumber == -1);
    CHECK(arduinoInterruptHandler == nullptr);
}

void testVSyncDesyncWaitsForNextPulse() {
    TestLeptonFLiR camera(SS, 8);
    camera.init(LeptonFLiR_CameraType_Lepton2);
    camera.configureNext(LeptonFLiR_CameraType_Lepton2,
                         LeptonFLiR_ImageMode_80x60_16bpp_164Brf,
                         LeptonFLiR_ImageOutputMode_GS16);

    std::vector<uint16_t> stream;
    appendPacket(stream, 0, rawPayload(0));
    appendPacket(stream, 5, rawPayload(0));
    for (int row = 0; row < 60; ++row)
        appendPacket(stream, static_cast<uint16_t>(row), rawPayload(static_cast<uint16_t>(0x2800 + row * 80)));
    SPI.clear();
    SPI.queue(stream);

    triggerInterrupt(8);
    CHECK(!camera.tryReadNextFrame());
    CHECK(!camera.tryReadNextFrame());
    triggerInterrupt(8);
    CHECK(camera.tryReadNextFrame());
    CHECK(camera.getImagePixelData(1, 3).std.value == static_cast<uint16_t>(0x2800 + 80 + 3));
}

void testVSyncDisabledFallsBackToVoSPI() {
    TestLeptonFLiR camera;
    camera.configureNext(LeptonFLiR_CameraType_Lepton2,
                         LeptonFLiR_ImageMode_80x60_16bpp_164Brf,
                         LeptonFLiR_ImageOutputMode_GS16);

    queue80RawFrame(0x3000);
    CHECK(camera.tryReadNextFrame());
    CHECK(camera.getImagePixelData(2, 7).std.value == static_cast<uint16_t>(0x3000 + 2 * 80 + 7));
}

} // namespace

int main() {
    testVSyncFrameReady();
    testVSyncDesyncWaitsForNextPulse();
    testVSyncDisabledFallsBackToVoSPI();

    if (failures) {
        std::cerr << failures << " consistency test(s) failed\n";
        return 1;
    }

    std::cout << "All LeptonFLiR consistency tests passed\n";
    return 0;
}
