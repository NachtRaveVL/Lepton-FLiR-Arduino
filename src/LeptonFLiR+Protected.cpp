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

    if (!_nextFrame || !_nextFrameNeedsUpdate)
        return;

    uint32_t value = 0;
    receiveCommand(cmdCode(LEP_CID_VID_OUTPUT_FORMAT, LEP_I2C_COMMAND_TYPE_GET), &value);
    if (_lastI2CError || _lastLepResult) {
        _nextFrame->imageMode = LeptonFLiR_ImageMode_Undefined;
        _nextFrame->outputMode = LeptonFLiR_ImageOutputMode_Undefined;
        return;
    }

    const LEP_VID_VIDEO_OUTPUT_FORMAT format = (LEP_VID_VIDEO_OUTPUT_FORMAT)value;

    switch(_cameraType) {
        case LeptonFLiR_CameraType_Lepton1:
        case LeptonFLiR_CameraType_Lepton1_5:
        case LeptonFLiR_CameraType_Lepton1_6:
        case LeptonFLiR_CameraType_Lepton2:
        case LeptonFLiR_CameraType_Lepton2_5:
            _nextFrame->imageMode = format == LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14 ? LeptonFLiR_ImageMode_80x60_16bpp_164Brf :
                                    format == LEP_VID_VIDEO_OUTPUT_FORMAT_RGB888 ? LeptonFLiR_ImageMode_80x60_24bpp_244Brf :
                                    LeptonFLiR_ImageMode_Undefined;
            break;

        case LeptonFLiR_CameraType_Lepton3:
        case LeptonFLiR_CameraType_Lepton3_5:
            _nextFrame->imageMode = format == LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14 ? LeptonFLiR_ImageMode_160x120_16bpp_164Brf :
                                    format == LEP_VID_VIDEO_OUTPUT_FORMAT_RGB888 ? LeptonFLiR_ImageMode_160x120_24bpp_244Brf :
                                    LeptonFLiR_ImageMode_Undefined;
            break;

        default:
            _nextFrame->imageMode = LeptonFLiR_ImageMode_Undefined;
            break;
    }

    if (_nextFrame->imageMode == LeptonFLiR_ImageMode_Undefined) {
        _nextFrame->outputMode = LeptonFLiR_ImageOutputMode_Undefined;
        _nextFrameNeedsUpdate = false;
        return;
    }

    _nextFrame->pclutEnabled = format == LEP_VID_VIDEO_OUTPUT_FORMAT_RGB888;

    value = 0;
    receiveCommand(cmdCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &value);
    if (_lastI2CError || _lastLepResult) return;
    _nextFrame->agcEnabled = value != 0;

    _nextFrame->tlinearEnabled = false;
    if (_cameraType == LeptonFLiR_CameraType_Lepton2_5 || _cameraType == LeptonFLiR_CameraType_Lepton3_5) {
        value = 0;
        receiveCommand(cmdCode(LEP_CID_RAD_TLINEAR_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &value);
        if (_lastI2CError || _lastLepResult) return;
        _nextFrame->tlinearEnabled = value != 0;
    }

    _nextFrame->telemetryMode = LeptonFLiR_TelemetryMode_Disabled;
    if (!_nextFrame->pclutEnabled) {
        value = 0;
        receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &value);
        if (_lastI2CError || _lastLepResult) return;

        if (value) {
            value = 0;
            receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_LOCATION, LEP_I2C_COMMAND_TYPE_GET), &value);
            if (_lastI2CError || _lastLepResult) return;
            _nextFrame->telemetryMode = value == LEP_TELEMETRY_LOCATION_HEADER ? LeptonFLiR_TelemetryMode_Header : LeptonFLiR_TelemetryMode_Footer;
        }
    }

    if (_nextFrame->pclutEnabled)
        _nextFrame->outputMode = LeptonFLiR_ImageOutputMode_RGB888;
    else if (_nextFrame->agcEnabled)
        _nextFrame->outputMode = LeptonFLiR_ImageOutputMode_GS8;
    else
        _nextFrame->outputMode = LeptonFLiR_ImageOutputMode_GS16;

    _nextFrameNeedsUpdate = false;
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
        byte *newFrameData = _frameData_orig ? roundUpRealloc16(_frameData_orig, frameDataSize) : roundUpMalloc16(frameDataSize);
        if (newFrameData) {
            _frameData_orig = newFrameData;
            _frameDataSize_orig = frameDataSize;
            _frameData = roundUpPtr16(_frameData_orig);
        }
    }

    if (offsetTableSize && nextFrame && !nextFrame->offsetTable) {
        nextFrame->offsetTable = new (std::nothrow) uint16_t[offsetTableSize];
        if (nextFrame->offsetTable)
            memset(nextFrame->offsetTable, 0, (size_t)offsetTableSize * sizeof(uint16_t));
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
    // Arduino's SPISettings does not expose the selected clock publicly, so keep
    // this approximation in sync with the divider rules used by the supported cores.
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
    if (!_frameData || !_nextFrame || line < 0) return NULL;

    int lineSize = 0;
    switch (_nextFrame->imageMode) {
        case LeptonFLiR_ImageMode_80x60_24bpp_244Brf:
        case LeptonFLiR_ImageMode_160x120_24bpp_244Brf:
            lineSize = 244;
            break;
        case LeptonFLiR_ImageMode_80x60_16bpp_164Brf:
        case LeptonFLiR_ImageMode_160x120_16bpp_164Brf:
            lineSize = 164;
            break;
        default:
            return NULL;
    }

    if (line >= _frameDataSize_orig / lineSize) return NULL;
    return (uint16_t *)((uintptr_t)_frameData + (uintptr_t)line * (uintptr_t)lineSize);
}

const byte *LeptonFLiR::getImageData(int row, int section) {
    if (!isImageDataAvailable() || !_lastFrame->offsetTable) return NULL;
    if (row < 0 || row >= getImageHeight()) return NULL;

    int index = row;
    if (getImageWidth() == 160) {
        if (section < 0 || section > 1) return NULL;
        index = row * 2 + section;
    }
    else if (section != 0)
        return NULL;

    return _lastFrame->imageData + _lastFrame->offsetTable[index];
}

const byte *LeptonFLiR::getTelemetryData(int row) {
    if (!isTelemetryDataAvailable() || row < 0) return NULL;

    int lineSize = 0;
    int telemetryLines = 0;
    switch (_lastFrame->imageMode) {
        case LeptonFLiR_ImageMode_80x60_16bpp_164Brf:
            lineSize = 164; telemetryLines = _lastFrame->telemetryMode != LeptonFLiR_TelemetryMode_Disabled ? 3 : 0;
            break;
        case LeptonFLiR_ImageMode_160x120_16bpp_164Brf:
            lineSize = 164; telemetryLines = _lastFrame->telemetryMode != LeptonFLiR_TelemetryMode_Disabled ? 4 : 0;
            break;
        default:
            return NULL;
    }

    return row < telemetryLines ? _lastFrame->telemetryData + row * lineSize : NULL;
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
