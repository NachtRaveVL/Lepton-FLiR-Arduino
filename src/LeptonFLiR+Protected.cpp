/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / Protected Members
*/

#include "LeptonFLiR.h"

LeptonFLiR::FrameSettings::FrameSettings(LeptonFLiR::FrameSettings* lastFrame, uint32_t frameNum)
    : frameNumber(frameNum),
      telemetryMode(lastFrame ? lastFrame->telemetryMode : LeptonFLiR_TelemetryMode_Disabled),
      agcEnabled(lastFrame ? lastFrame->agcEnabled : false),
      tlinearEnabled(lastFrame ? lastFrame->tlinearEnabled : false),
      pclutEnabled(lastFrame ? lastFrame->pclutEnabled : false),
      imageMode(lastFrame ? lastFrame->imageMode : LeptonFLiR_ImageMode_Undefined),
      outputMode(lastFrame ? lastFrame->outputMode : LeptonFLiR_ImageOutputMode_Undefined),
      offsetTable(NULL), imageData(NULL), telemetryData(NULL)
{ }

LeptonFLiR::FrameSettings::~FrameSettings() {
    if (offsetTable) { delete [] offsetTable; offsetTable = NULL; }
}

LeptonFLiR::FrameSettings* LeptonFLiR::getNextFrame() {
    if (_isReadingNextFrame) return NULL;

    if (_nextFrameNeedsUpdate)
        updateNextFrame();

    return _nextFrame;
}

void LeptonFLiR::updateNextFrame() {
    if (!_nextFrame)
        advanceNextFrame();

    if (_nextFrame && _nextFrameNeedsUpdate) {
        LEP_VID_VIDEO_OUTPUT_FORMAT format = vid_getOutputFormat();

        switch(_cameraType) {
            case LeptonFLiR_CameraType_Lepton1:
            case LeptonFLiR_CameraType_Lepton1_5:
            case LeptonFLiR_CameraType_Lepton1_6:
            case LeptonFLiR_CameraType_Lepton2:
            case LeptonFLiR_CameraType_Lepton2_5: {
                switch (format) {
                    case LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14:
                        _nextFrame->imageMode = LeptonFLiR_ImageMode_80x60_16bpp_164Brf;
                        break;
                    case LEP_VID_VIDEO_OUTPUT_FORMAT_RGB888:
                        _nextFrame->imageMode = LeptonFLiR_ImageMode_80x60_24bpp_244Brf;
                        break;
                    default:
                        _nextFrame->imageMode = LeptonFLiR_ImageMode_Undefined;
                        break;
                }
            } break;

            case LeptonFLiR_CameraType_Lepton3:
            case LeptonFLiR_CameraType_Lepton3_5: {
                switch (format) {
                    case LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14:
                        _nextFrame->imageMode = LeptonFLiR_ImageMode_160x120_16bpp_164Brf;
                        break;
                    case LEP_VID_VIDEO_OUTPUT_FORMAT_RGB888:
                        _nextFrame->imageMode = LeptonFLiR_ImageMode_160x120_24bpp_244Brf;
                        break;
                    default:
                        _nextFrame->imageMode = LeptonFLiR_ImageMode_Undefined;
                        break;
                }
            } break;

            default:
                _nextFrame->imageMode = LeptonFLiR_ImageMode_Undefined;
                break;
        }

        // TODO: Continue update for other values. -NR
        // LeptonFLiR_TelemetryMode telemetryMode
        // bool agcEnabled
        // bool tlinearEnabled
        // bool pclutEnabled
        // LeptonFLiR_ImageMode imageMode
        // LeptonFLiR_ImageOutputMode outputMode

        _nextFrameNeedsUpdate = false;
    }
}

void LeptonFLiR::advanceNextFrame() {
    if (_lastFrame) { delete _lastFrame; _lastFrame = NULL; }
    if (_nextFrame) { _lastFrame = _nextFrame; _nextFrame = NULL; }
    if (!_nextFrame) {
        _nextFrame = new (std::nothrow) LeptonFLiR::FrameSettings(_lastFrame, _frameCounter++);
        if (!_lastFrame) _nextFrameNeedsUpdate = true; // Safety
    }
}

void LeptonFLiR::prepareNextFrame() {
    LeptonFLiR::FrameSettings* nextFrame = getNextFrame();
    int frameDataSize = getSPIFrameTotalSize();
    int offsetTableSize = getSPIFrameImageLines();

    if (frameDataSize && (!_frameData_orig || _frameDataSize_orig != frameDataSize)) {
        _frameData_orig = _frameData_orig ? roundUpRealloc16(_frameData_orig, frameDataSize) : roundUpMalloc16(frameDataSize);
        _frameDataSize_orig = frameDataSize;
        _frameData = roundUpPtr16(_frameData_orig);
    }

    if (offsetTableSize && nextFrame && !nextFrame->offsetTable) {
        nextFrame->offsetTable = new (std::nothrow) uint16_t[offsetTableSize];
    }
}

uint32_t LeptonFLiR::getNextFrameNumber() {
    LeptonFLiR::FrameSettings* nextFrame = getNextFrame();
    return nextFrame ? nextFrame->frameNumber : 0;
}

LeptonFLiR_ImageMode LeptonFLiR::getNextImageMode() {
    LeptonFLiR::FrameSettings* nextFrame = getNextFrame();
    return nextFrame ? nextFrame->imageMode : LeptonFLiR_ImageMode_Undefined;
}

LeptonFLiR_ImageOutputMode LeptonFLiR::getNextImageOutputMode() {
    LeptonFLiR::FrameSettings* nextFrame = getNextFrame();
    return nextFrame ? nextFrame->outputMode : LeptonFLiR_ImageOutputMode_Undefined;
}

LeptonFLiR_TelemetryMode LeptonFLiR::getNextTelemetryMode() {
    LeptonFLiR::FrameSettings* nextFrame = getNextFrame();
    return nextFrame ? nextFrame->telemetryMode : LeptonFLiR_TelemetryMode_Disabled;
}

bool LeptonFLiR::getNextAGCEnabled() {
    LeptonFLiR::FrameSettings* nextFrame = getNextFrame();
    return nextFrame ? nextFrame->agcEnabled : false;
}

bool LeptonFLiR::getNextTLinearEnabled() {
    LeptonFLiR::FrameSettings* nextFrame = getNextFrame();
    return nextFrame ? nextFrame->tlinearEnabled : false;
}

bool LeptonFLiR::getNextPseudoColorLUTEnabled() {
    LeptonFLiR::FrameSettings* nextFrame = getNextFrame();
    return nextFrame ? nextFrame->pclutEnabled : false;
}

int LeptonFLiR::getSPIClockDivisor() {
    // TODO: Investigate other means to get SPI speed directly from SPISettings. -NR
    int divisor = 2;
#ifdef __SAM3X8E__
    // Arduino Due has non-power-of-2 capable divisors
    while (divisor < 128 && F_CPU / (float)divisor > LEPFLIR_SPI_MAX_SPEED + FLT_EPSILON)
        ++divisor;
#else
    while (divisor < 128 && F_CPU / (float)divisor > LEPFLIR_SPI_MAX_SPEED + FLT_EPSILON)
        divisor *= 2;
#endif
    return divisor;
}

int LeptonFLiR::getSPIFrameLineSize() {
    switch(getNextImageMode()) {
        case LeptonFLiR_ImageMode_80x60_24bpp_244Brf:
        case LeptonFLiR_ImageMode_160x120_24bpp_244Brf:
            return 244;
        case LeptonFLiR_ImageMode_80x60_16bpp_164Brf:
        case LeptonFLiR_ImageMode_160x120_16bpp_164Brf:
            return 164;
        default:
            return 0;
    }
}

int LeptonFLiR::getSPIFrameLineSize16() {
    switch(getNextImageMode()) {
        case LeptonFLiR_ImageMode_80x60_24bpp_244Brf:
        case LeptonFLiR_ImageMode_160x120_24bpp_244Brf:
            return 122;
        case LeptonFLiR_ImageMode_80x60_16bpp_164Brf:
        case LeptonFLiR_ImageMode_160x120_16bpp_164Brf:
            return 82;
        default:
            return 0;
    }
}

int LeptonFLiR::getSPIFrameDataSize() {
    switch(getNextImageMode()) {
        case LeptonFLiR_ImageMode_80x60_24bpp_244Brf:
        case LeptonFLiR_ImageMode_160x120_24bpp_244Brf:
            return 240;
        case LeptonFLiR_ImageMode_80x60_16bpp_164Brf:
        case LeptonFLiR_ImageMode_160x120_16bpp_164Brf:
            return 160;
        default:
            return 0;
    }
}

int LeptonFLiR::getSPIFrameDataSize16() {
    switch(getNextImageMode()) {
        case LeptonFLiR_ImageMode_80x60_24bpp_244Brf:
        case LeptonFLiR_ImageMode_160x120_24bpp_244Brf:
            return 120;
        case LeptonFLiR_ImageMode_80x60_16bpp_164Brf:
        case LeptonFLiR_ImageMode_160x120_16bpp_164Brf:
            return 80;
        default:
            return 0;
    }
}

int LeptonFLiR::getSPIFrameTelemetryLines() {
    switch(getNextImageMode()) {
        case LeptonFLiR_ImageMode_80x60_16bpp_164Brf:
            return getNextTelemetryMode() != LeptonFLiR_TelemetryMode_Disabled ? 3 : 0;
        case LeptonFLiR_ImageMode_160x120_16bpp_164Brf:
            return getNextTelemetryMode() != LeptonFLiR_TelemetryMode_Disabled ? 4 : 0;
        case LeptonFLiR_ImageMode_80x60_24bpp_244Brf:
        case LeptonFLiR_ImageMode_160x120_24bpp_244Brf:
        default:
            return 0;
    }
}

int LeptonFLiR::getSPIFrameImageLines() {
    switch(_cameraType) {
        case LeptonFLiR_CameraType_Lepton1:
        case LeptonFLiR_CameraType_Lepton1_5:
        case LeptonFLiR_CameraType_Lepton1_6:
        case LeptonFLiR_CameraType_Lepton2:
        case LeptonFLiR_CameraType_Lepton2_5:
            return 60;
        case LeptonFLiR_CameraType_Lepton3:
        case LeptonFLiR_CameraType_Lepton3_5:
            return 240;
        default:
            return 0;
    }
}

int LeptonFLiR::getSPIFrameTotalLines() {
    return getSPIFrameTelemetryLines() + getSPIFrameImageLines();
}

int LeptonFLiR::getSPIFrameTotalSize() {
    return getSPIFrameLineSize() * getSPIFrameTotalLines();
}

int LeptonFLiR::getSPIFrameTotalSize16() {
    return getSPIFrameLineSize16() * getSPIFrameTotalLines();
}

uint16_t *LeptonFLiR::getSPIFrameData(int line) {
    return (uint16_t *)((uintptr_t)_frameData + line*getSPIFrameLineSize());
}

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

static void printSPIFrame(uint16_t *spiFrame) {
    Serial.print("ID: 0x");
    Serial.print(spiFrame[0], HEX);
    Serial.print(" CRC: 0x");
    Serial.print(spiFrame[1], HEX);
    Serial.print(" Data: ");
    for (int i = 0; i < 5; ++i) {
        Serial.print(i ? "-0x" : "0x");
        Serial.print(spiFrame[i + 2], HEX);
    }
    Serial.print("...");
    int offset = getSPIFrameDataSize16() - 5;
    for (int i = 0; i < 5; ++i) {
        Serial.print(i ? "-0x" : "0x");
        Serial.print(spiFrame[offset + i + 2], HEX);
    }
    Serial.println("");
}

#endif

const byte *LeptonFLiR::getImageData(int row, int section) {
    // TODO: Write get image data (section). -NR
    return NULL;
}

const byte *LeptonFLiR::getTelemetryData(int row) {
    return isTelemetryDataAvailable() ? (const byte *)((uintptr_t)_lastFrame->telemetryData + row*getSPIFrameLineSize()) : NULL;
}

float LeptonFLiR::kelvin100ToCelsius(uint16_t kelvin100) {
    float kelvin = (kelvin100 / 100) + ((kelvin100 % 100) * 0.01f);
    return kelvin - 273.15f;
}

float LeptonFLiR::kelvin100ToFahrenheit(uint16_t kelvin100) {
    float kelvin = (kelvin100 / 100) + ((kelvin100 % 100) * 0.01f);
    return roundf((((kelvin * 9.0f) / 5.0f) - 459.67f) * 100.0f) / 100.0f;
}

float LeptonFLiR::kelvin100ToKelvin(uint16_t kelvin100) {
    return (kelvin100 / 100) + ((kelvin100 % 100) * 0.01f);
}

uint16_t LeptonFLiR::celsiusToKelvin100(float celsius) {
    float kelvin = celsius + 273.15f;
    return (uint16_t)roundf(kelvin * 100.0f);
}

uint16_t LeptonFLiR::fahrenheitToKelvin100(float fahrenheit) {
    float kelvin = ((fahrenheit + 459.67f) * 5.0f) / 9.0f;
    return (uint16_t)roundf(kelvin * 100.0f);
}

uint16_t LeptonFLiR::kelvinToKelvin100(float kelvin) {
    return (uint16_t)roundf(kelvin * 100.0f);
}
