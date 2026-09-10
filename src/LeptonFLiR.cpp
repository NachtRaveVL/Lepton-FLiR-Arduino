/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR Main
*/

#include "LeptonFLiR.h"

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

static String LEPFLIR_makeAssertMsg(String msg, const char *file, const char *func, int line)
{
    return String(F("Assertion Failure: ")) + String(file) + String(':') + String(line) + String(F(" in ")) + String(func) + String(':') + String(' ') + msg;
}

void LEPFLIR_softAssert(bool cond, String msg, const char *file, const char *func, int line)
{
    if (!cond) {
        String message = LEPFLIR_makeAssertMsg(msg, file, func, line);
        if (Serial) { Serial.println(message); }
    }
}

void LEPFLIR_hardAssert(bool cond, String msg, const char *file, const char *func, int line)
{
    if (!cond) {
        String message = String(F("HARD ")) + LEPFLIR_makeAssertMsg(msg, file, func, line);
        if (Serial) {
            Serial.println(message);
            Serial.flush();
        }
        yield(); delay(10);
        abort();
    }
}

#endif // /ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT


LeptonFLiR *LeptonFLiR::_isrVSyncOwner = NULL;

void LeptonFLiR::handleVSyncInterrupt() {
    if (_isrVSyncOwner)
        _isrVSyncOwner->_vsyncFrameReady = true;
}

#ifndef LEPFLIR_USE_SOFTWARE_I2C

LeptonFLiR::LeptonFLiR(byte spiCSPin, byte isrVSyncPin, TwoWire& i2cWire, uint32_t i2cSpeed)
    : _spiCSPin(spiCSPin), _isrVSyncPin(isrVSyncPin),
      _i2cWire(&i2cWire),
      _i2cSpeed(i2cSpeed),
      _spiSettings(SPISettings(LEPFLIR_SPI_MAX_SPEED, MSBFIRST, SPI_MODE3)),
      _spiDMAEnabled(false),
      _cameraType(LeptonFLiR_CameraType_Undefined),
      _tempMode(LeptonFLiR_TemperatureMode_Undefined),
      _frameData(NULL), _frameData_orig(NULL), _frameDataSize_orig(0),
      _imageOutput(NULL), _imageOutput_orig(NULL), _imageOutputSize_orig(0),
      _telemetryOutput(NULL),
      _frameCounter(0),
      _lastFrame(NULL), _nextFrame(NULL), _nextFrameNeedsUpdate(true),
      _isReadingNextFrame(false), _vsyncFrameReady(false),
      _lastI2CError(0), _lastLepResult(0)
{ }

LeptonFLiR::LeptonFLiR(TwoWire& i2cWire, uint32_t i2cSpeed, byte spiCSPin, byte isrVSyncPin)
    : _spiCSPin(spiCSPin), _isrVSyncPin(isrVSyncPin),
      _i2cWire(&i2cWire),
      _i2cSpeed(i2cSpeed),
      _spiSettings(SPISettings(LEPFLIR_SPI_MAX_SPEED, MSBFIRST, SPI_MODE3)),
      _spiDMAEnabled(false),
      _cameraType(LeptonFLiR_CameraType_Undefined),
      _tempMode(LeptonFLiR_TemperatureMode_Undefined),
      _frameData(NULL), _frameData_orig(NULL), _frameDataSize_orig(0),
      _imageOutput(NULL), _imageOutput_orig(NULL), _imageOutputSize_orig(0),
      _telemetryOutput(NULL),
      _frameCounter(0),
      _lastFrame(NULL), _nextFrame(NULL), _nextFrameNeedsUpdate(true),
      _isReadingNextFrame(false), _vsyncFrameReady(false),
      _lastI2CError(0), _lastLepResult(0)
{ }

#else

LeptonFLiR::LeptonFLiR(byte spiCSPin, byte isrVSyncPin)
    : _spiCSPin(spiCSPin), _isrVSyncPin(isrVSyncPin),
      _spiSettings(SPISettings(LEPFLIR_SPI_MAX_SPEED, MSBFIRST, SPI_MODE3)),
      _spiDMAEnabled(false),
      _cameraType(LeptonFLiR_CameraType_Undefined),
      _tempMode(LeptonFLiR_TemperatureMode_Undefined),
      _frameData(NULL), _frameData_orig(NULL), _frameDataSize_orig(0),
      _imageOutput(NULL), _imageOutput_orig(NULL), _imageOutputSize_orig(0),
      _telemetryOutput(NULL),
      _frameCounter(0),
      _lastFrame(NULL), _nextFrame(NULL), _nextFrameNeedsUpdate(true),
      _isReadingNextFrame(false), _vsyncFrameReady(false),
      _lastI2CError(0), _lastLepResult(0),
      _readBytes(0)
{ }

#endif // /ifndef LEPFLIR_USE_SOFTWARE_I2C

LeptonFLiR::~LeptonFLiR() {
    if (_isrVSyncPin != DISABLED && _isrVSyncOwner == this) {
        detachInterrupt(digitalPinToInterrupt(_isrVSyncPin));
        _isrVSyncOwner = NULL;
    }
    _frameData = NULL;
    if (_frameData_orig) { free(_frameData_orig);  _frameData_orig = NULL; _frameDataSize_orig = 0; }
    _imageOutput = NULL;
    if (_imageOutput_orig) { free(_imageOutput_orig); _imageOutput_orig = NULL; _imageOutputSize_orig = 0; }
    if (_telemetryOutput) { delete _telemetryOutput; _telemetryOutput = NULL; }
    if (_lastFrame) { delete _lastFrame; _lastFrame = NULL; }
    if (_nextFrame) { delete _nextFrame; _nextFrame = NULL; }
}

void LeptonFLiR::init(LeptonFLiR_CameraType cameraType, LeptonFLiR_TemperatureMode tempMode) {
    _cameraType = (LeptonFLiR_CameraType)min(max((int)cameraType, 0), (int)LeptonFLiR_CameraType_Count - 1);
    _tempMode = (LeptonFLiR_TemperatureMode)min(max((int)tempMode, 0), (int)LeptonFLiR_TemperatureMode_Count - 1);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    const int spiDivisor = getSPIClockDivisor();
    const float spiSpeed = F_CPU / (float)spiDivisor;
    Serial.print(F("LeptonFLiR::init cameraType: v"));
    Serial.print(getCameraVersion(), 1);
    Serial.print(F(", tempMode: "));
    Serial.print(getTemperatureSymbol());
    Serial.print(F(", spiCSPin: "));
    Serial.print(_spiCSPin);
    Serial.print(F(", isrVSyncPin: "));
    if (_isrVSyncPin != DISABLED) {
        Serial.print(_isrVSyncPin);
        Serial.print(F(" <on-rising>"));
    } else
        Serial.print(F("<disabled>"));
    Serial.print(F(", i2cWire#: "));
    Serial.print(getWireInterfaceNumber());
    Serial.print(F(", i2cSpeed: "));
    Serial.print(roundf(getI2CSpeed() / 1000.0f)); Serial.print(F("kHz"));
    Serial.print(F(", spiSpeed: "));
    Serial.print(roundf(spiSpeed / 1000.0f) / 1000.0f);
    Serial.print(F("MHz (SPI_CLOCK_DIV")); Serial.print(spiDivisor); Serial.print(F(")"));
    if (spiSpeed < LEPFLIR_SPI_MIN_SPEED - FLT_EPSILON)
        Serial.print(F(" <speed too low>"));
    else if (spiSpeed > LEPFLIR_SPI_MAX_SPEED + FLT_EPSILON)
        Serial.print(F(" <speed too high>"));
    else if (spiSpeed < LEPFLIR_SPI_OPTIMAL_MIN_SPEED - FLT_EPSILON)
        Serial.print(F(" <speed sub-optimal>"));
    Serial.println("");
#endif

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif

    // Redundant in many cases, but done for safety
    pinMode(_spiCSPin, OUTPUT);
    digitalWrite(_spiCSPin, HIGH);

    if (_isrVSyncPin != DISABLED) {
        pinMode(_isrVSyncPin, INPUT);
        _vsyncFrameReady = false;
        if (_isrVSyncOwner && _isrVSyncOwner != this && _isrVSyncOwner->_isrVSyncPin != DISABLED)
            detachInterrupt(digitalPinToInterrupt(_isrVSyncOwner->_isrVSyncPin));
        _isrVSyncOwner = this;
        attachInterrupt(digitalPinToInterrupt(_isrVSyncPin), handleVSyncInterrupt, RISING);
    }
}

byte LeptonFLiR::getChipSelectPin() {
    return _spiCSPin;
}

byte LeptonFLiR::getISRVSyncPin() {
    return _isrVSyncPin;
}

uint32_t LeptonFLiR::getI2CSpeed() {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cSpeed;
#else
#if I2C_FASTMODE || F_CPU >= 16000000
    return 400000;
#else
    return 100000;
#endif
#endif // ifndef LEPFLIR_USE_SOFTWARE_I2C
}

LeptonFLiR_CameraType LeptonFLiR::getCameraType() {
    return _cameraType;
}

LeptonFLiR_TemperatureMode LeptonFLiR::getTemperatureMode() {
    return _tempMode;
}

int LeptonFLiR::getImageWidth() {
    switch (_cameraType) {
        case LeptonFLiR_CameraType_Lepton1:
        case LeptonFLiR_CameraType_Lepton1_5:
        case LeptonFLiR_CameraType_Lepton1_6:
        case LeptonFLiR_CameraType_Lepton2:
        case LeptonFLiR_CameraType_Lepton2_5:
            return 80;
        case LeptonFLiR_CameraType_Lepton3:
        case LeptonFLiR_CameraType_Lepton3_5:
            return 160;
        default:
            return 0;
    }
}

bool LeptonFLiR::setSPIDMAEnabled(bool enabled) {
#ifdef SPI_HAS_TRANSFER_ASYNC
    _spiDMAEnabled = enabled;
    return true;
#else
    _spiDMAEnabled = false;
    return !enabled;
#endif
}

bool LeptonFLiR::getSPIDMAEnabled() {
    return _spiDMAEnabled;
}

bool LeptonFLiR::isSPIDMAAvailable() {
#ifdef SPI_HAS_TRANSFER_ASYNC
    return true;
#else
    return false;
#endif
}

int LeptonFLiR::getImageHeight() {
    switch (_cameraType) {
        case LeptonFLiR_CameraType_Lepton1:
        case LeptonFLiR_CameraType_Lepton1_5:
        case LeptonFLiR_CameraType_Lepton1_6:
        case LeptonFLiR_CameraType_Lepton2:
        case LeptonFLiR_CameraType_Lepton2_5:
            return 60;
        case LeptonFLiR_CameraType_Lepton3:
        case LeptonFLiR_CameraType_Lepton3_5:
            return 120;
        default:
            return 0;
    }
}

uint32_t LeptonFLiR::getFrameNumber() {
    return _lastFrame ? _lastFrame->frameNumber : 0;
}

LeptonFLiR_TelemetryMode LeptonFLiR::getTelemetryMode() {
    return _lastFrame ? _lastFrame->telemetryMode : LeptonFLiR_TelemetryMode_Disabled;
}

bool LeptonFLiR::getAGCEnabled() {
    return _lastFrame ? _lastFrame->agcEnabled : false;
}

bool LeptonFLiR::getTLinearEnabled() {
    return _lastFrame ? _lastFrame->tlinearEnabled : false;
}

bool LeptonFLiR::getPseudoColorLUTEnabled() {
    return _lastFrame ? _lastFrame->pclutEnabled : false;
}

bool LeptonFLiR::isImageDataAvailable() {
    return !_isReadingNextFrame && _lastFrame && _lastFrame->imageData;
}
LeptonFLiR_ImageMode LeptonFLiR::getImageMode() {
    return _lastFrame ? _lastFrame->imageMode : LeptonFLiR_ImageMode_Undefined;
}

int LeptonFLiR::getImageBpp() {
    switch (getImageMode()) {
        case LeptonFLiR_ImageMode_80x60_24bpp_244Brf:
        case LeptonFLiR_ImageMode_160x120_24bpp_244Brf:
            return 3;
        case LeptonFLiR_ImageMode_80x60_16bpp_164Brf:
        case LeptonFLiR_ImageMode_160x120_16bpp_164Brf:
            return 2;
        default:
            return 0;
    }
}

static byte getSPIDataByte(const byte *data, int offset) {
    const uint16_t *words = (const uint16_t *)data;
    const uint16_t word = words[offset >> 1];
    return (offset & 1) ? lowByte(word) : highByte(word);
}

LeptonFLiR_PixelData LeptonFLiR::getImagePixelData(int row, int col) {
    LeptonFLiR_PixelData pixel = {};
    if (!isImageDataAvailable() || row < 0 || row >= getImageHeight() || col < 0 || col >= getImageWidth())
        return pixel;

    const int section = getImageWidth() == 160 ? col / 80 : 0;
    const int sectionCol = getImageWidth() == 160 ? col % 80 : col;
    const byte *imageData = getImageData(row, section);
    if (!imageData) return pixel;

    switch (getImageMode()) {
        case LeptonFLiR_ImageMode_80x60_16bpp_164Brf:
        case LeptonFLiR_ImageMode_160x120_16bpp_164Brf: {
            const uint16_t value = ((const uint16_t *)imageData)[sectionCol];
            if (getAGCEnabled() && !_lastFrame->agc14Bit) {
                pixel.agc._res = highByte(value);
                pixel.agc.value = lowByte(value);
            }
            else if (getTLinearEnabled())
                pixel.tlinear.value = value;
            else
                pixel.std.value = value;
        } break;

        case LeptonFLiR_ImageMode_80x60_24bpp_244Brf:
        case LeptonFLiR_ImageMode_160x120_24bpp_244Brf: {
            const int offset = sectionCol * 3;
            pixel.pclut.red = getSPIDataByte(imageData, offset);
            pixel.pclut.green = getSPIDataByte(imageData, offset + 1);
            pixel.pclut.blue = getSPIDataByte(imageData, offset + 2);
        } break;

        default:
            break;
    }

    return pixel;
}

LeptonFLiR_ImageOutputMode LeptonFLiR::getImageOutputMode() {
    return _lastFrame ? _lastFrame->outputMode : LeptonFLiR_ImageOutputMode_Undefined;
}

int LeptonFLiR::getImageOutputBpp() {
    switch (getImageOutputMode()) {
        case LeptonFLiR_ImageOutputMode_GS8:
            return 1;
        case LeptonFLiR_ImageOutputMode_GS16:
            return 2;
        case LeptonFLiR_ImageOutputMode_RGB565:
            return 2;
        case LeptonFLiR_ImageOutputMode_RGB888:
            return 3;
        default:
            return 0;
    }
}

int LeptonFLiR::getImageOutputPitch() {
    return roundUpVal16(getImageWidth() * getImageOutputBpp());
}

int LeptonFLiR::getImageOutputTotalSize() {
    return getImageHeight() * getImageOutputPitch();
}

byte *LeptonFLiR::getImageOutputData() {
    if (!isImageDataAvailable()) return NULL;

    const int outputSize = getImageOutputTotalSize();
    if (outputSize <= 0) return NULL;

    if (!_imageOutput_orig || _imageOutputSize_orig != outputSize) {
        byte *newOutput = _imageOutput_orig ? roundUpRealloc16(_imageOutput_orig, outputSize) : roundUpMalloc16(outputSize);
        if (!newOutput) return NULL;
        _imageOutput_orig = newOutput;
        _imageOutputSize_orig = outputSize;
        _imageOutput = roundUpPtr16(_imageOutput_orig);
    }

    memset(_imageOutput, 0, (size_t)outputSize);
    getImageOutputData(_imageOutput, getImageOutputPitch());
    return _imageOutput;
}

void LeptonFLiR::getImageOutputData(byte *image, int pitch) {
    if (!image || !isImageDataAvailable()) return;

    const int width = getImageWidth();
    const int height = getImageHeight();
    const int bpp = getImageOutputBpp();
    const int rowBytes = width * bpp;
    if (!width || !height || !bpp || pitch < rowBytes) return;

    for (int row = 0; row < height; ++row) {
        byte *dst = image + row * pitch;
        for (int col = 0; col < width; ++col) {
            const LeptonFLiR_PixelData pixel = getImagePixelData(row, col);
            switch (getImageOutputMode()) {
                case LeptonFLiR_ImageOutputMode_GS8:
                    dst[col] = pixel.agc.value;
                    break;

                case LeptonFLiR_ImageOutputMode_GS16: {
                    const uint16_t value = getTLinearEnabled() ? pixel.tlinear.value : pixel.std.value;
                    memcpy(dst + col * 2, &value, sizeof(value));
                } break;

                case LeptonFLiR_ImageOutputMode_RGB565: {
                    const uint16_t value = (uint16_t)(((uint16_t)(pixel.pclut.red & 0xF8) << 8) |
                                                      ((uint16_t)(pixel.pclut.green & 0xFC) << 3) |
                                                      ((uint16_t)pixel.pclut.blue >> 3));
                    memcpy(dst + col * 2, &value, sizeof(value));
                } break;

                case LeptonFLiR_ImageOutputMode_RGB888:
                    dst[col * 3] = pixel.pclut.red;
                    dst[col * 3 + 1] = pixel.pclut.green;
                    dst[col * 3 + 2] = pixel.pclut.blue;
                    break;

                default:
                    return;
            }
        }
    }
}

bool LeptonFLiR::isTelemetryDataAvailable() {
    return !_isReadingNextFrame && _lastFrame && _lastFrame->telemetryData;
}

uint32_t LeptonFLiR::getTelemetryFrameCounter() {
    if (!isTelemetryDataAvailable()) return 0;
    const uint16_t *telemetryData_A = (const uint16_t *)getTelemetryData(0);

    return ((uint32_t)telemetryData_A[20] << 16) | (uint32_t)telemetryData_A[21];
}

bool LeptonFLiR::getTelemetryShouldRunFFCNormalization() {
    if (!isTelemetryDataAvailable()) return false;
    const uint16_t *telemetryData_A = (const uint16_t *)getTelemetryData(0);

    const uint_fast8_t ffcState = (telemetryData_A[4] & 0x0030) >> 4;
    return (telemetryData_A[4] & 0x0008) && ffcState != 2;
}

bool LeptonFLiR::getTelemetryAGCEnabled() {
    if (!isTelemetryDataAvailable()) return false;
    const uint16_t *telemetryData_A = (const uint16_t *)getTelemetryData(0);

    return telemetryData_A[4] & 0x1000;
}

LeptonFLiR_TelemetryData* LeptonFLiR::getTelemetryOutputData() {
    if (!isTelemetryDataAvailable()) return NULL;
    if (!_telemetryOutput)
        _telemetryOutput = new (std::nothrow) LeptonFLiR_TelemetryData();
    if (!_telemetryOutput) return NULL;
    getTelemetryOutputData(_telemetryOutput);
    return _telemetryOutput;
}

void LeptonFLiR::getTelemetryOutputData(LeptonFLiR_TelemetryData *telemetry) {
    if (!telemetry || !isTelemetryDataAvailable()) return;
    memset(telemetry, 0, sizeof(*telemetry));
    const uint16_t *telemetryData_A = (const uint16_t *)getTelemetryData(0);
    //const uint16_t *telemetryData_B = (const uint16_t *)getTelemetryData(1);
    //const uint16_t *telemetryData_C = (const uint16_t *)getTelemetryData(2);

    telemetry->revisionMajor = highByte(telemetryData_A[0]);
    telemetry->revisionMinor = lowByte(telemetryData_A[0]);

    telemetry->cameraUptime = ((uint32_t)telemetryData_A[1] << 16) | (uint32_t)telemetryData_A[2];

    telemetry->ffcDesired = telemetryData_A[4] & 0x0008;
    // Engineering datasheet Rev 203 Table 3: 0=never, 1=imminent, 2=active, 3=done.
    const LeptonFLiR_TelemetryFFCState ffcStates[] = {
        LeptonFLiR_TelemetryFFCState_NeverCommanded, LeptonFLiR_TelemetryFFCState_Imminent,
        LeptonFLiR_TelemetryFFCState_InProgress, LeptonFLiR_TelemetryFFCState_Complete
    };
    telemetry->ffcState = ffcStates[(telemetryData_A[4] & 0x0030) >> 4];
    telemetry->agcEnabled = telemetryData_A[4] & 0x1000;
    telemetry->shutdownImminent = telemetryData_A[3] & 0x0010;

    LeptonFLiR::wordsToHexString(&telemetryData_A[5], 8, telemetry->serialNumber, sizeof(telemetry->serialNumber));
    LeptonFLiR::wordsToHexString(&telemetryData_A[13], 4, telemetry->softwareRevision, sizeof(telemetry->softwareRevision));

    telemetry->frameCounter = ((uint32_t)telemetryData_A[20] << 16) | (uint32_t)telemetryData_A[21];
    telemetry->frameMean = telemetryData_A[22];

    telemetry->fpaTemperature = kelvin100ToTemperature(telemetryData_A[24]);
    telemetry->housingTemperature = kelvin100ToTemperature(telemetryData_A[26]);

    telemetry->lastFFCTime = ((uint32_t)telemetryData_A[30] << 16) | (uint32_t)telemetryData_A[31];
    telemetry->fpaTempAtLastFFC = kelvin100ToTemperature(telemetryData_A[29]);
    telemetry->housingTempAtLastFFC = kelvin100ToTemperature(telemetryData_A[32]);

    telemetry->agcRegion.startRow = telemetryData_A[34];
    telemetry->agcRegion.startCol = telemetryData_A[35];
    telemetry->agcRegion.endRow = telemetryData_A[36];
    telemetry->agcRegion.endCol = telemetryData_A[37];

    telemetry->agcClipHigh = telemetryData_A[38];
    telemetry->agcClipLow = telemetryData_A[39];

    telemetry->vidFormat = (LEP_VID_VIDEO_OUTPUT_FORMAT)(((uint32_t)telemetryData_A[72] << 16) | (uint32_t)telemetryData_A[73]);
    telemetry->log2FFC = telemetryData_A[74];

    const uint16_t *telemetryData_B = (const uint16_t *)getTelemetryData(1);
    if (telemetryData_B) {
        telemetry->sceneEmissivity = telemetryData_B[19];
        telemetry->bgTemperature = kelvin100ToTemperature(telemetryData_B[20]);
        telemetry->atmoTau = telemetryData_B[21];
        telemetry->atmoTemperature = kelvin100ToTemperature(telemetryData_B[22]);
        telemetry->windowTau = telemetryData_B[23];
        telemetry->windowReflTau = telemetryData_B[24];
        telemetry->windowTemperature = kelvin100ToTemperature(telemetryData_B[25]);
        telemetry->windowReflTemperature = kelvin100ToTemperature(telemetryData_B[26]);
    }

    const uint16_t *telemetryData_C = (const uint16_t *)getTelemetryData(2);
    if (telemetryData_C) {
        telemetry->gainMode = (LeptonFLiR_TelemetryGainMode)telemetryData_C[5];
        telemetry->effGainMode = (LeptonFLiR_TelemetryGainMode)telemetryData_C[6];
        telemetry->gainModeSwitchDesired = telemetryData_C[7] != 0;
        telemetry->radGainModeSwitchHtLTemp = kelvin100ToTemperature(celsiusToKelvin100((float)telemetryData_C[8]));
        telemetry->radGainModeSwitchLtHTemp = kelvin100ToTemperature(celsiusToKelvin100((float)telemetryData_C[9]));
        telemetry->tlinearGainModeSwitchHtLTemp = kelvin100ToTemperature(kelvinToKelvin100((float)telemetryData_C[10]));
        telemetry->tlinearGainModeSwitchLtHTemp = kelvin100ToTemperature(kelvinToKelvin100((float)telemetryData_C[11]));
    }
}

//#define LEPFLIR_ENABLE_FRAME_PACKET_DEBUG_OUTPUT    1

bool LeptonFLiR::tryReadNextFrame() {
    if (_isReadingNextFrame) return false;
    if (_isrVSyncPin != DISABLED) {
        if (!_vsyncFrameReady) return false;
        _vsyncFrameReady = false;
    }

    prepareNextFrame();
    FrameSettings *nextFrame = _nextFrame;
    const int lineSize = getSPIFrameLineSize();
    const int lineSize16 = getSPIFrameLineSize16();
    const int imageLines = getSPIFrameImageLines();
    const int telemetryLines = getSPIFrameTelemetryLines();

    if (!nextFrame || !_frameData || !nextFrame->offsetTable || !lineSize || !lineSize16 || !imageLines)
        return false;

    _isReadingNextFrame = true;
    nextFrame->imageData = NULL;
    nextFrame->telemetryData = NULL;
    if (_lastFrame) {
        _lastFrame->imageData = NULL;
        _lastFrame->telemetryData = NULL;
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::tryReadNextFrame"));
#endif

    const bool segmented = getImageWidth() == 160;
    const int segmentCount = segmented ? 4 : 1;
    const int packetsPerSegment = segmented ? 60 + (telemetryLines ? 1 : 0) : 60 + telemetryLines;
    bool success = false;

    SPI.beginTransaction(_spiSettings);
    digitalWrite(_spiCSPin, HIGH);

    // Work from the current stream position first. A hard re-sync is only needed
    // if packet sequencing is actually lost. When VSYNC is used, wait for the
    // next frame-ready pulse instead of retrying against the same frame period.
    const int maxAttempts = _isrVSyncPin != DISABLED ? 1 : 2;
    for (int attempt = 0; attempt < maxAttempts && !success; ++attempt) {
        if (attempt)
            delay(186);

        success = true;
        int packetBudget = 2048;
        digitalWrite(_spiCSPin, LOW);

        if (segmented) {
            for (int expectedSegment = 1; expectedSegment <= segmentCount && success; ++expectedSegment) {
                bool segmentFound = false;

                while (!segmentFound && packetBudget > 0) {
                    const int baseLine = (expectedSegment - 1) * packetsPerSegment;
                    uint16_t *spiFrame = getSPIFrameData(baseLine);
                    if (!spiFrame) {
                        success = false;
                        break;
                    }

                    // Packet zero marks the start of a segment. Discard packets and
                    // the tail of any segment already in progress are ignored here.
                    bool packetZeroFound = false;
                    while (packetBudget-- > 0) {
                        SPI_transfer16(spiFrame, lineSize16);
                        const uint16_t id = spiFrame[0];
                        if ((id & 0x0F00) != 0x0F00 && (id & 0x0FFF) == 0) {
                            packetZeroFound = true;
                            break;
                        }
                    }

                    if (!packetZeroFound) {
                        success = false;
                        break;
                    }

                    int segmentNumber = -1;
                    for (int packet = 1; packet < packetsPerSegment; ++packet) {
                        spiFrame = getSPIFrameData(baseLine + packet);
                        if (!spiFrame || packetBudget-- <= 0) {
                            success = false;
                            break;
                        }

                        SPI_transfer16(spiFrame, lineSize16);
                        const uint16_t id = spiFrame[0];
                        if ((id & 0x0F00) == 0x0F00 || (id & 0x0FFF) != (uint16_t)packet) {
                            success = false;
                            break;
                        }

                        if (packet == 20)
                            segmentNumber = (id >> 12) & 0x7;
                    }

                    if (!success)
                        break;

                    if (segmentNumber < 0 || segmentNumber > 4) {
                        success = false;
                        break;
                    }

                    if (expectedSegment == 1) {
                        // Lepton 3 inserts invalid segments with TTT=0 between
                        // unique frames. Segments 2-4 can also be encountered if
                        // the read started partway through a valid frame.
                        segmentFound = segmentNumber == 1;
                    }
                    else {
                        if (segmentNumber != expectedSegment) {
                            success = false;
                            break;
                        }
                        segmentFound = true;
                    }
                }

                if (!segmentFound)
                    success = false;
            }
        }
        else {
            uint16_t *spiFrame = getSPIFrameData(0);
            bool packetZeroFound = false;

            while (spiFrame && packetBudget-- > 0) {
                SPI_transfer16(spiFrame, lineSize16);
                const uint16_t id = spiFrame[0];
                if ((id & 0x0F00) != 0x0F00 && (id & 0x0FFF) == 0) {
                    packetZeroFound = true;
                    break;
                }
            }

            if (!packetZeroFound)
                success = false;

            for (int packet = 1; packet < packetsPerSegment && success; ++packet) {
                spiFrame = getSPIFrameData(packet);
                if (!spiFrame || packetBudget-- <= 0) {
                    success = false;
                    break;
                }

                SPI_transfer16(spiFrame, lineSize16);
                const uint16_t id = spiFrame[0];
                if ((id & 0x0F00) == 0x0F00 || (id & 0x0FFF) != (uint16_t)packet)
                    success = false;
            }
        }

        digitalWrite(_spiCSPin, HIGH);

        if (success) {
            int imageIndex = 0;
            int telemetryIndex = 0;
            uint16_t telemetryOffset = 0;

            for (int segment = 1; segment <= segmentCount; ++segment) {
                for (int packet = 0; packet < packetsPerSegment; ++packet) {
                    const int bufferLine = (segment - 1) * packetsPerSegment + packet;
                    const uint16_t *spiFrame = getSPIFrameData(bufferLine);
                    if (!spiFrame) {
                        success = false;
                        break;
                    }

                    bool isTelemetry = false;
                    if (telemetryLines) {
                        if (!segmented)
                            isTelemetry = nextFrame->telemetryMode == LeptonFLiR_TelemetryMode_Header ? packet < 3 : packet >= 60;
                        else if (nextFrame->telemetryMode == LeptonFLiR_TelemetryMode_Header)
                            isTelemetry = segment == 1 && packet < 4;
                        else
                            isTelemetry = segment == 4 && packet >= 57;
                    }

                    const byte *payload = (const byte *)(spiFrame + 2);
                    const uint16_t payloadOffset = (uint16_t)(payload - (_frameData + 4));
                    if (isTelemetry) {
                        if (telemetryIndex == 0) telemetryOffset = payloadOffset;
                        ++telemetryIndex;
                    }
                    else {
                        if (imageIndex >= imageLines) {
                            success = false;
                            break;
                        }
                        nextFrame->offsetTable[imageIndex++] = payloadOffset;
                    }
                }

                if (!success)
                    break;
            }

            if (imageIndex != imageLines || telemetryIndex != telemetryLines)
                success = false;

            if (success) {
                nextFrame->imageData = _frameData + 4;
                if (telemetryLines)
                    nextFrame->telemetryData = _frameData + 4 + telemetryOffset;
            }
        }
    }

    if (!success && _isrVSyncPin != DISABLED) {
        // A VSYNC pulse does not reset VoSPI after a sequencing failure.
        // CS is already high; keep SCK idle for >185 ms before awaiting a fresh pulse.
        delay(186);
        _vsyncFrameReady = false;
    }

    SPI.endTransaction();
    _isReadingNextFrame = false;

    if (!success)
        return false;

    advanceNextFrame();
    return true;
}
