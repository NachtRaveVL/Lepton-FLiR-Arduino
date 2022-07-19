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


#ifndef LEPFLIR_USE_SOFTWARE_I2C

LeptonFLiR::LeptonFLiR(byte spiCSPin, byte isrVSyncPin, TwoWire& i2cWire, uint32_t i2cSpeed)
    : _spiCSPin(spiCSPin), _isrVSyncPin(isrVSyncPin),
      _i2cWire(&i2cWire),
      _i2cSpeed(i2cSpeed),
      _spiSettings(SPISettings(LEPFLIR_SPI_MAX_SPEED, MSBFIRST, SPI_MODE3)),
      _cameraType(LeptonFLiR_CameraType_Undefined),
      _tempMode(LeptonFLiR_TemperatureMode_Undefined),
      _frameData(NULL), _frameData_orig(NULL), _frameDataSize_orig(0),
      _imageOutput(NULL), _imageOutput_orig(NULL), _imageOutputSize_orig(0),
      _telemetryOutput(NULL),
      _frameCounter(0),
      _lastFrame(NULL), _nextFrame(NULL), _nextFrameNeedsUpdate(true),
      _isReadingNextFrame(false),
      _lastI2CError(0), _lastLepResult(0)
{ }

LeptonFLiR::LeptonFLiR(TwoWire& i2cWire, uint32_t i2cSpeed, byte spiCSPin, byte isrVSyncPin)
    : _spiCSPin(spiCSPin), _isrVSyncPin(isrVSyncPin),
      _i2cWire(&i2cWire),
      _i2cSpeed(i2cSpeed),
      _spiSettings(SPISettings(LEPFLIR_SPI_MAX_SPEED, MSBFIRST, SPI_MODE3)),
      _cameraType(LeptonFLiR_CameraType_Undefined),
      _tempMode(LeptonFLiR_TemperatureMode_Undefined),
      _frameData(NULL), _frameData_orig(NULL), _frameDataSize_orig(0),
      _imageOutput(NULL), _imageOutput_orig(NULL), _imageOutputSize_orig(0),
      _telemetryOutput(NULL),
      _frameCounter(0),
      _lastFrame(NULL), _nextFrame(NULL), _nextFrameNeedsUpdate(true),
      _isReadingNextFrame(false),
      _lastI2CError(0), _lastLepResult(0)
{ }

#else

LeptonFLiR::LeptonFLiR(byte spiCSPin, byte isrVSyncPin)
    : _spiCSPin(spiCSPin), _isrVSyncPin(isrVSyncPin),
      _readBytes(0),
      _spiSettings(SPISettings(LEPFLIR_SPI_MAX_SPEED, MSBFIRST, SPI_MODE3)),
      _cameraType(LeptonFLiR_CameraType_Undefined),
      _tempMode(LeptonFLiR_TemperatureMode_Undefined),
      _frameData(NULL), _frameData_orig(NULL), _frameDataSize_orig(0),
      _imageOutput(NULL), _imageOutput_orig(NULL), _imageOutputSize_orig(0),
      _telemetryOutput(NULL),
      _frameCounter(0),
      _lastFrame(NULL), _nextFrame(NULL), _nextFrameNeedsUpdate(true),
      _isReadingNextFrame(false),
      _lastI2CError(0), _lastLepResult(0)
{ }

#endif // /ifndef LEPFLIR_USE_SOFTWARE_I2C

LeptonFLiR::~LeptonFLiR() {
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
    const float spiSpeed = F_CPU / (const float)spiDivisor;
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
        // TODO: Write/enable ISR. -NR
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

LeptonFLiR_PixelData LeptonFLiR::getImagePixelData(int row, int col) {
    // TODO: Write get image pixel data. -NR
    return LeptonFLiR_PixelData();
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
    // TODO: Write image output creation. -NR
    return NULL;
}

void LeptonFLiR::getImageOutputData(byte *image, int pitch) {
    // TODO: Write image output copy. -NR
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

    uint_fast8_t ffcState = (telemetryData_A[4] & 0x0018) >> 3;
    if (lowByte(telemetryData_A[0]) >= 9 && ffcState >= 1)
        ffcState -= 1;

    return (telemetryData_A[4] & 0x0004) && ffcState != (uint_fast8_t)LeptonFLiR_TelemetryFFCState_InProgress;
}

bool LeptonFLiR::getTelemetryAGCEnabled() {
    if (!isTelemetryDataAvailable()) return false;
    const uint16_t *telemetryData_A = (const uint16_t *)getTelemetryData(0);

    return telemetryData_A[4] & 0x0800;
}

LeptonFLiR_TelemetryData* LeptonFLiR::getTelemetryOutputData() {
    // TODO: Write telemetry output creation. -NR
    return NULL;
}

void LeptonFLiR::getTelemetryOutputData(LeptonFLiR_TelemetryData *telemetry) {
    if (!isTelemetryDataAvailable()) return;
    const uint16_t *telemetryData_A = (const uint16_t *)getTelemetryData(0);
    //const uint16_t *telemetryData_B = (const uint16_t *)getTelemetryData(1);
    //const uint16_t *telemetryData_C = (const uint16_t *)getTelemetryData(2);
    // TODO: Verify Telem B and C lines are always next to A line in SPI buffer memory. -NR

    telemetry->revisionMajor = lowByte(telemetryData_A[0]);
    telemetry->revisionMinor = highByte(telemetryData_A[0]);

    telemetry->cameraUptime = ((uint32_t)telemetryData_A[1] << 16) | (uint32_t)telemetryData_A[2];

    telemetry->ffcDesired = telemetryData_A[4] & 0x0004;
    uint_fast8_t ffcState = (telemetryData_A[4] & 0x0018) >> 3;
    if (telemetry->revisionMajor >= 9 && ffcState >= 1)
        ffcState -= 1;
    telemetry->ffcState = (LeptonFLiR_TelemetryFFCState)ffcState;
    telemetry->agcEnabled = telemetryData_A[4] & 0x0800;
    telemetry->shutdownImminent = telemetryData_A[3] & 0x0010;

    LeptonFLiR::wordsToHexString(&telemetryData_A[5], 8, telemetry->serialNumber, 24);
    LeptonFLiR::wordsToHexString(&telemetryData_A[13], 4, telemetry->softwareRevision, 12);

    telemetry->frameCounter = ((uint32_t)telemetryData_A[20] << 16) | (uint32_t)telemetryData_A[21];
    telemetry->frameMean = telemetryData_A[22];

    telemetry->fpaTemperature = kelvin100ToTemperature(telemetryData_A[24]);
    telemetry->housingTemperature = kelvin100ToTemperature(telemetryData_A[26]);

    telemetry->lastFFCTime = ((uint32_t)telemetryData_A[30] << 16) | (uint32_t)telemetryData_A[31];
    telemetry->fpaTempAtLastFFC = kelvin100ToTemperature(telemetryData_A[29]);
    telemetry->housingTempAtLastFFC = kelvin100ToTemperature(telemetryData_A[32]);

    telemetry->agcRegion.startRow = telemetryData_A[34];
    telemetry->agcRegion.startCol = telemetryData_A[35];
    telemetry->agcRegion.endCol = telemetryData_A[36];
    telemetry->agcRegion.endRow = telemetryData_A[37];

    telemetry->agcClipHigh = telemetryData_A[38];
    telemetry->agcClipLow = telemetryData_A[39];

    telemetry->log2FFC = telemetryData_A[74];

    // TODO: Do Telem B and C line getters below. -NR
    // LEP_VID_VIDEO_OUTPUT_FORMAT vidFormat
    // uint16_t sceneEmissivity
    // uint16_t atmoTau
    // uint16_t windowTau
    // uint16_t windowReflTau
    // float bgTemperature
    // float atmoTemperature
    // float windowTemperature
    // float windowReflTemperature
    // LeptonFLiR_TelemetryGainMode gainMode
    // LeptonFLiR_TelemetryGainMode effGainMode
    // bool gainModeSwitchDesired
    // float radGainModeSwitchHtLTemp
    // float radGainModeSwitchLtHTemp
    // float tlinearGainModeSwitchHtLTemp
}

//#define LEPFLIR_ENABLE_FRAME_PACKET_DEBUG_OUTPUT    1

bool LeptonFLiR::tryReadNextFrame() {
    if (!_isReadingNextFrame) {
        _isReadingNextFrame = true;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.println(F("LeptonFLiR::tryReadNextFrame"));
#endif
        // TODO

    }
    return false;
}
