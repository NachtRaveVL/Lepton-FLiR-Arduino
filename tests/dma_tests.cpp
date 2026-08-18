#include "LeptonFLiR.h"
#include "SPI.h"

#include <cstdint>
#include <iostream>
#include <vector>

namespace {

int failures = 0;
#define CHECK(expr) do { if (!(expr)) { std::cerr << "FAIL " << __LINE__ << ": " #expr "\n"; ++failures; } } while (0)

class TestLeptonFLiR : public LeptonFLiR {
public:
    using LeptonFLiR::LeptonFLiR;

    void configure() {
        _cameraType = LeptonFLiR_CameraType_Lepton2;
        _tempMode = LeptonFLiR_TemperatureMode_Celsius;
        if (!_nextFrame)
            _nextFrame = new FrameSettings(_lastFrame, _frameCounter++);
        _nextFrame->imageMode = LeptonFLiR_ImageMode_80x60_16bpp_164Brf;
        _nextFrame->outputMode = LeptonFLiR_ImageOutputMode_GS16;
        _nextFrame->telemetryMode = LeptonFLiR_TelemetryMode_Disabled;
        _nextFrame->agcEnabled = false;
        _nextFrame->tlinearEnabled = false;
        _nextFrame->pclutEnabled = false;
        _nextFrameNeedsUpdate = false;
    }
};

void appendPacket(std::vector<uint16_t>& stream, int row) {
    stream.push_back(static_cast<uint16_t>(row));
    stream.push_back(0);
    for (int col = 0; col < 80; ++col)
        stream.push_back(static_cast<uint16_t>(0x7000 + row * 80 + col));
}

} // namespace

int main() {
    TestLeptonFLiR camera;
    camera.configure();
    CHECK(camera.isSPIDMAAvailable());
    CHECK(camera.setSPIDMAEnabled());
    CHECK(camera.getSPIDMAEnabled());

    std::vector<uint16_t> stream;
    for (int row = 0; row < 60; ++row)
        appendPacket(stream, row);

    SPI.clear();
    SPI.queue(stream);
    CHECK(camera.tryReadNextFrame());
    CHECK(SPI.asyncTransferCount >= 60);
    CHECK(camera.getImagePixelData(12, 34).std.value == static_cast<uint16_t>(0x7000 + 12 * 80 + 34));

    TestLeptonFLiR fallbackCamera;
    fallbackCamera.configure();
    CHECK(fallbackCamera.setSPIDMAEnabled());

    stream.clear();
    for (int row = 0; row < 60; ++row)
        appendPacket(stream, row);

    SPI.clear();
    SPI.queue(stream);
    SPI.asyncShouldFail = true;
    CHECK(fallbackCamera.tryReadNextFrame());
    CHECK(SPI.asyncTransferCount == 1);
    CHECK(!fallbackCamera.getSPIDMAEnabled());
    CHECK(fallbackCamera.getImagePixelData(7, 19).std.value == static_cast<uint16_t>(0x7000 + 7 * 80 + 19));

    if (failures) return 1;
    std::cout << "LeptonFLiR DMA SPI tests passed\n";
    return 0;
}
