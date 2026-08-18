#include "LeptonFLiR.h"
#include "SPI.h"

#include <cmath>
#include <cstdint>
#include <cstring>
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
                       LeptonFLiR_ImageOutputMode outputMode,
                       LeptonFLiR_TelemetryMode telemetryMode = LeptonFLiR_TelemetryMode_Disabled,
                       bool agcEnabled = false,
                       bool tlinearEnabled = false,
                       bool pclutEnabled = false) {
        _cameraType = cameraType;
        _tempMode = LeptonFLiR_TemperatureMode_Celsius;
        if (!_nextFrame)
            _nextFrame = new FrameSettings(_lastFrame, _frameCounter++);
        _nextFrame->imageMode = imageMode;
        _nextFrame->outputMode = outputMode;
        _nextFrame->telemetryMode = telemetryMode;
        _nextFrame->agcEnabled = agcEnabled;
        _nextFrame->tlinearEnabled = tlinearEnabled;
        _nextFrame->pclutEnabled = pclutEnabled;
        _nextFrameNeedsUpdate = false;
    }

    bool isReading() const { return _isReadingNextFrame; }
};

void appendPacket(std::vector<uint16_t>& stream, uint16_t id, const std::vector<uint16_t>& payload) {
    stream.push_back(id);
    stream.push_back(0);
    stream.insert(stream.end(), payload.begin(), payload.end());
}

std::vector<uint16_t> rawPayload(uint16_t base) {
    std::vector<uint16_t> payload(80);
    for (int i = 0; i < 80; ++i)
        payload[i] = static_cast<uint16_t>(base + i);
    return payload;
}

std::vector<uint16_t> rgbPayload(byte r, byte g, byte b) {
    std::vector<byte> bytes;
    bytes.reserve(240);
    for (int i = 0; i < 80; ++i) {
        bytes.push_back(static_cast<byte>(r + i));
        bytes.push_back(g);
        bytes.push_back(b);
    }

    std::vector<uint16_t> words(120);
    for (int i = 0; i < 120; ++i)
        words[i] = static_cast<uint16_t>((static_cast<uint16_t>(bytes[i * 2]) << 8) | bytes[i * 2 + 1]);
    return words;
}

void test80RawFrame() {
    TestLeptonFLiR camera;
    camera.configureNext(LeptonFLiR_CameraType_Lepton2,
                         LeptonFLiR_ImageMode_80x60_16bpp_164Brf,
                         LeptonFLiR_ImageOutputMode_GS16);

    std::vector<uint16_t> stream;
    for (int row = 0; row < 60; ++row)
        appendPacket(stream, static_cast<uint16_t>(row), rawPayload(static_cast<uint16_t>(0x1000 + row * 80)));
    SPI.clear();
    SPI.queue(stream);

    CHECK(camera.tryReadNextFrame());
    CHECK(camera.isImageDataAvailable());
    CHECK(camera.getImageWidth() == 80);
    CHECK(camera.getImageHeight() == 60);
    CHECK(camera.getImageBpp() == 2);
    CHECK(camera.getImageOutputBpp() == 2);
    CHECK(camera.getImagePixelData(10, 5).std.value == static_cast<uint16_t>(0x1000 + 10 * 80 + 5));

    byte *output = camera.getImageOutputData();
    CHECK(output != nullptr);
    CHECK(camera.getImageOutputPitch() == 160);
    uint16_t outputPixel = 0;
    std::memcpy(&outputPixel, output + 10 * camera.getImageOutputPitch() + 5 * 2, sizeof(outputPixel));
    CHECK(outputPixel == static_cast<uint16_t>(0x1000 + 10 * 80 + 5));
}

void test80AGCFrame() {
    TestLeptonFLiR camera;
    camera.configureNext(LeptonFLiR_CameraType_Lepton2,
                         LeptonFLiR_ImageMode_80x60_16bpp_164Brf,
                         LeptonFLiR_ImageOutputMode_GS8,
                         LeptonFLiR_TelemetryMode_Disabled,
                         true);

    std::vector<uint16_t> stream;
    for (int row = 0; row < 60; ++row) {
        std::vector<uint16_t> payload(80);
        for (int col = 0; col < 80; ++col)
            payload[col] = static_cast<uint16_t>((row + col) & 0xFF);
        appendPacket(stream, static_cast<uint16_t>(row), payload);
    }
    SPI.clear();
    SPI.queue(stream);

    CHECK(camera.tryReadNextFrame());
    CHECK(camera.getImagePixelData(7, 9).agc.value == 16);
    byte *output = camera.getImageOutputData();
    CHECK(output != nullptr);
    CHECK(output[7 * camera.getImageOutputPitch() + 9] == 16);
}

void test80RGBFrame() {
    TestLeptonFLiR camera;
    camera.configureNext(LeptonFLiR_CameraType_Lepton2,
                         LeptonFLiR_ImageMode_80x60_24bpp_244Brf,
                         LeptonFLiR_ImageOutputMode_RGB888,
                         LeptonFLiR_TelemetryMode_Disabled,
                         false, false, true);

    std::vector<uint16_t> stream;
    for (int row = 0; row < 60; ++row)
        appendPacket(stream, static_cast<uint16_t>(row), rgbPayload(static_cast<byte>(row), 0x55, 0xAA));
    SPI.clear();
    SPI.queue(stream);

    CHECK(camera.tryReadNextFrame());
    const LeptonFLiR_PixelData pixel = camera.getImagePixelData(4, 7);
    CHECK(pixel.pclut.red == 11);
    CHECK(pixel.pclut.green == 0x55);
    CHECK(pixel.pclut.blue == 0xAA);
    CHECK(camera.getImageOutputBpp() == 3);

    byte *output = camera.getImageOutputData();
    CHECK(output != nullptr);
    const int offset = 4 * camera.getImageOutputPitch() + 7 * 3;
    CHECK(output[offset] == 11);
    CHECK(output[offset + 1] == 0x55);
    CHECK(output[offset + 2] == 0xAA);
}

void test160SegmentMapping() {
    TestLeptonFLiR camera;
    camera.configureNext(LeptonFLiR_CameraType_Lepton3,
                         LeptonFLiR_ImageMode_160x120_16bpp_164Brf,
                         LeptonFLiR_ImageOutputMode_GS16);

    std::vector<uint16_t> stream;
    int halfLine = 0;
    for (int segment = 1; segment <= 4; ++segment) {
        for (int packet = 0; packet < 60; ++packet, ++halfLine) {
            const uint16_t id = static_cast<uint16_t>(packet == 20 ? (segment << 12) | packet : packet);
            appendPacket(stream, id, std::vector<uint16_t>(80, static_cast<uint16_t>(0x2000 + halfLine)));
        }
    }
    SPI.clear();
    SPI.queue(stream);

    CHECK(camera.tryReadNextFrame());
    CHECK(camera.getImageWidth() == 160);
    CHECK(camera.getImageHeight() == 120);
    CHECK(camera.getImagePixelData(30, 0).std.value == 0x203C);
    CHECK(camera.getImagePixelData(30, 80).std.value == 0x203D);
    CHECK(camera.getImagePixelData(119, 159).std.value == 0x20EF);
}

void testTelemetryHeader() {
    TestLeptonFLiR camera;
    camera.configureNext(LeptonFLiR_CameraType_Lepton2_5,
                         LeptonFLiR_ImageMode_80x60_16bpp_164Brf,
                         LeptonFLiR_ImageOutputMode_GS16,
                         LeptonFLiR_TelemetryMode_Header);

    std::vector<uint16_t> stream;
    for (int packet = 0; packet < 63; ++packet) {
        std::vector<uint16_t> payload(80, 0);
        if (packet == 0) {
            payload[20] = 0x1234;
            payload[21] = 0x5678;
            payload[24] = 30015;
            payload[72] = 0;
            payload[73] = LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14;
        }
        else if (packet == 1) {
            payload[19] = 7000;
            payload[20] = 29515;
            payload[21] = 8000;
            payload[22] = 29615;
            payload[23] = 7900;
            payload[24] = 100;
            payload[25] = 29715;
            payload[26] = 29815;
        }
        else if (packet == 2) {
            payload[5] = LeptonFLiR_TelemetryGainMode_Auto;
            payload[6] = LeptonFLiR_TelemetryGainMode_Low;
            payload[7] = 1;
            payload[8] = 115;
            payload[9] = 85;
            payload[10] = 388;
            payload[11] = 358;
        }
        else {
            payload = rawPayload(static_cast<uint16_t>(0x3000 + (packet - 3) * 80));
        }
        appendPacket(stream, static_cast<uint16_t>(packet), payload);
    }
    SPI.clear();
    SPI.queue(stream);

    CHECK(camera.tryReadNextFrame());
    CHECK(camera.isTelemetryDataAvailable());
    CHECK(camera.getTelemetryFrameCounter() == 0x12345678UL);
    CHECK(camera.getImagePixelData(0, 0).std.value == 0x3000);

    LeptonFLiR_TelemetryData *telemetry = camera.getTelemetryOutputData();
    CHECK(telemetry != nullptr);
    CHECK(telemetry->sceneEmissivity == 7000);
    CHECK(telemetry->atmoTau == 8000);
    CHECK(telemetry->gainMode == LeptonFLiR_TelemetryGainMode_Auto);
    CHECK(telemetry->effGainMode == LeptonFLiR_TelemetryGainMode_Low);
    CHECK(telemetry->gainModeSwitchDesired);
    CHECK(std::fabs(telemetry->bgTemperature - 22.0f) < 0.01f);
    CHECK(std::fabs(telemetry->tlinearGainModeSwitchHtLTemp - 114.85f) < 0.02f);
}

void test160TelemetryHeader() {
    TestLeptonFLiR camera;
    camera.configureNext(LeptonFLiR_CameraType_Lepton3_5,
                         LeptonFLiR_ImageMode_160x120_16bpp_164Brf,
                         LeptonFLiR_ImageOutputMode_GS16,
                         LeptonFLiR_TelemetryMode_Header);

    std::vector<uint16_t> stream;
    int halfLine = 0;
    for (int segment = 1; segment <= 4; ++segment) {
        for (int packet = 0; packet < 61; ++packet) {
            const uint16_t id = static_cast<uint16_t>(packet == 20 ? (segment << 12) | packet : packet);
            const bool telemetry = segment == 1 && packet < 4;
            std::vector<uint16_t> payload(80, 0);

            if (telemetry) {
                if (packet == 0) {
                    payload[20] = 0xABCD;
                    payload[21] = 0xEF01;
                }
            }
            else {
                payload.assign(80, static_cast<uint16_t>(0x6000 + halfLine++));
            }

            appendPacket(stream, id, payload);
        }
    }

    SPI.clear();
    SPI.queue(stream);

    CHECK(camera.tryReadNextFrame());
    CHECK(camera.getTelemetryFrameCounter() == 0xABCDEF01UL);
    CHECK(camera.getImagePixelData(0, 0).std.value == 0x6000);
    CHECK(camera.getImagePixelData(119, 159).std.value == 0x60EF);
}

void testResyncRetry() {
    TestLeptonFLiR camera;
    camera.configureNext(LeptonFLiR_CameraType_Lepton2,
                         LeptonFLiR_ImageMode_80x60_16bpp_164Brf,
                         LeptonFLiR_ImageOutputMode_GS16);

    std::vector<uint16_t> stream;
    appendPacket(stream, 0, rawPayload(0));
    appendPacket(stream, 5, rawPayload(0));
    for (int row = 0; row < 60; ++row)
        appendPacket(stream, static_cast<uint16_t>(row), rawPayload(static_cast<uint16_t>(0x4000 + row * 80)));

    SPI.clear();
    SPI.queue(stream);

    CHECK(camera.tryReadNextFrame());
    CHECK(camera.getImagePixelData(2, 3).std.value == static_cast<uint16_t>(0x4000 + 2 * 80 + 3));
}

void test160SkipsInvalidSegments() {
    TestLeptonFLiR camera;
    camera.configureNext(LeptonFLiR_CameraType_Lepton3,
                         LeptonFLiR_ImageMode_160x120_16bpp_164Brf,
                         LeptonFLiR_ImageOutputMode_GS16);

    std::vector<uint16_t> stream;
    for (int invalid = 0; invalid < 2; ++invalid) {
        for (int packet = 0; packet < 60; ++packet) {
            const uint16_t id = static_cast<uint16_t>(packet);
            appendPacket(stream, id, std::vector<uint16_t>(80, 0xDEAD));
        }
    }

    int halfLine = 0;
    for (int segment = 1; segment <= 4; ++segment) {
        for (int packet = 0; packet < 60; ++packet, ++halfLine) {
            const uint16_t id = static_cast<uint16_t>(packet == 20 ? (segment << 12) | packet : packet);
            appendPacket(stream, id, std::vector<uint16_t>(80, static_cast<uint16_t>(0x5000 + halfLine)));
        }
    }

    SPI.clear();
    SPI.queue(stream);

    CHECK(camera.tryReadNextFrame());
    CHECK(camera.getImagePixelData(0, 0).std.value == 0x5000);
    CHECK(camera.getImagePixelData(119, 159).std.value == 0x50EF);
}

void testFailedReadCanRetry() {
    TestLeptonFLiR camera;
    camera.configureNext(LeptonFLiR_CameraType_Lepton2,
                         LeptonFLiR_ImageMode_80x60_16bpp_164Brf,
                         LeptonFLiR_ImageOutputMode_GS16);

    std::vector<uint16_t> badStream;
    appendPacket(badStream, 5, rawPayload(0));
    SPI.clear();
    SPI.queue(badStream);
    CHECK(!camera.tryReadNextFrame());
    CHECK(!camera.isReading());

    std::vector<uint16_t> goodStream;
    for (int row = 0; row < 60; ++row)
        appendPacket(goodStream, static_cast<uint16_t>(row), rawPayload(static_cast<uint16_t>(row * 80)));
    SPI.clear();
    SPI.queue(goodStream);
    CHECK(camera.tryReadNextFrame());
}

} // namespace

int main() {
    test80RawFrame();
    test80AGCFrame();
    test80RGBFrame();
    test160SegmentMapping();
    testTelemetryHeader();
    test160TelemetryHeader();
    testResyncRetry();
    test160SkipsInvalidSegments();
    testFailedReadCanRetry();

    if (failures) {
        std::cerr << failures << " test(s) failed\n";
        return 1;
    }

    std::cout << "All LeptonFLiR host tests passed\n";
    return 0;
}
