/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR Main
*/

#include "LeptonFLiR.h"

#ifndef digitalWriteFast
static void csEnableFuncDef(byte pin) { digitalWrite(pin, LOW); }
static void csDisableFuncDef(byte pin) { digitalWrite(pin, HIGH); }
#else
static void csEnableFuncDef(byte pin) { digitalWriteFast(pin, LOW); }
static void csDisableFuncDef(byte pin) { digitalWriteFast(pin, HIGH); }
#endif

#ifndef LEPFLIR_USE_SOFTWARE_I2C

LeptonFLiR::LeptonFLiR(byte spiCSPin, byte isrVSyncPin, TwoWire& i2cWire, uint32_t i2cSpeed)
    : _spiCSPin(spiCSPin), _isrVSyncPin(isrVSyncPin),
      _i2cWire(&i2cWire),
      _i2cSpeed(i2cSpeed),
      _spiSettings(SPISettings(LEPFLIR_SPI_MAX_SPEED, MSBFIRST, SPI_MODE3)),
      _cameraType(LeptonFLiR_CameraType_Undefined),
      _tempMode(LeptonFLiR_TemperatureMode_Undefined),
      _csEnableFunc(csEnableFuncDef), _csDisableFunc(csDisableFuncDef),
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
      _csEnableFunc(csEnableFuncDef), _csDisableFunc(csDisableFuncDef),
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
      _csEnableFunc(csEnableFuncDef), _csDisableFunc(csDisableFuncDef),
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
    _cameraType = (LeptonFLiR_CameraType)constrain((int)cameraType, 0, (int)LeptonFLiR_CameraType_Count - 1);
    _tempMode = (LeptonFLiR_TemperatureMode)constrain((int)tempMode, 0, (int)LeptonFLiR_TemperatureMode_Count - 1);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    const int spiDivisor = getSPIClockDivisor();
    const float spiSpeed = F_CPU / (const float)spiDivisor;
    Serial.print("LeptonFLiR::init cameraType: v");
    Serial.print(getCameraVersion(), 1);
    Serial.print(", tempMode: ");
    Serial.print(getTemperatureSymbol());
    Serial.print(", spiCSPin: ");
    Serial.print(_spiCSPin);
    Serial.print(", isrVSyncPin: ");
    if (_isrVSyncPin != DISABLED) {
        Serial.print(_isrVSyncPin);
        Serial.print(" (on-rising)");
    } else
        Serial.print("<disabled>");
    Serial.print(", i2cWire#: ");
    Serial.print(getWireInterfaceNumber());
    Serial.print(", i2cSpeed: ");
    Serial.print(roundf(getI2CSpeed() / 1000.0f)); Serial.print("kHz");
    Serial.print(", spiSpeed: ");
    Serial.print(roundf(spiSpeed / 1000.0f) / 1000.0f);
    Serial.print("MHz (SPI_CLOCK_DIV"); Serial.print(spiDivisor); Serial.print(")");
    if (spiSpeed < LEPFLIR_SPI_MIN_SPEED - FLT_EPSILON)
        Serial.print(" <speed too low>");
    else if (spiSpeed > LEPFLIR_SPI_MAX_SPEED + FLT_EPSILON)
        Serial.print(" <speed too high>");
    else if (spiSpeed < LEPFLIR_SPI_OPTIMAL_MIN_SPEED - FLT_EPSILON)
        Serial.print(" <speed sub-optimal>");
    Serial.println("");
#endif

    i2cWire_begin();

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif

#ifndef __SAM3X8E__ // Arduino Due's SPI library handles its own CS latching
    pinMode(_spiCSPin, OUTPUT);
    _csDisableFunc(_spiCSPin);
#endif

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

void LeptonFLiR::setFastCSFuncs(digitalWriteFunc csEnableFunc, digitalWriteFunc csDisableFunc) {
    _csEnableFunc = csEnableFunc ? csEnableFunc : csEnableFuncDef;
    _csDisableFunc = csDisableFunc ? csDisableFunc : csDisableFuncDef;
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
    return !_isReadingNextFrame && _lastFrame && _lastFrame->_imageData;
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
}

void LeptonFLiR::getTelemetryOutputData(LeptonFLiR_TelemetryData *telemetry) {
    if (!isTelemetryDataAvailable()) return;
    const uint16_t *telemetryData_A = (const uint16_t *)getTelemetryData(0);
    const uint16_t *telemetryData_B = (const uint16_t *)getTelemetryData(1);
    const uint16_t *telemetryData_C = (const uint16_t *)getTelemetryData(2);
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

    // TODO: Do remaining in this list. -NR
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
        Serial.println("LeptonFLiR::tryReadNextFrame");
#endif

        bool agc8Enabled;
        LEP_SYS_TELEMETRY_LOCATION telemetryLocation;

        {   bool telemetryEnabled, cameraBooted, stateErrors = false;
            uint32_t value = 0;

            receiveCommand(cmdCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &value);
            agc8Enabled = value;
            stateErrors = stateErrors || _lastI2CError || _lastLepResult;

            if (agc8Enabled) {
                receiveCommand(cmdCode(LEP_CID_AGC_HEQ_SCALE_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &value);
                agc8Enabled = (value == (uint32_t)LEP_AGC_SCALE_TO_8_BITS);
                stateErrors = stateErrors || _lastI2CError || _lastLepResult;
            }

            receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &value);
            telemetryEnabled = value;
            stateErrors = stateErrors || _lastI2CError || _lastLepResult;

            if (telemetryEnabled) {
                receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_LOCATION, LEP_I2C_COMMAND_TYPE_GET), &value);
                telemetryLocation = (LEP_SYS_TELEMETRY_LOCATION)value;
                stateErrors = stateErrors || _lastI2CError || _lastLepResult;
            }

            uint16_t status; readRegister(LEP_I2C_STATUS_REG, &status);
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
            checkForErrors();
#endif
            cameraBooted = (status & LEP_I2C_STATUS_BOOT_MODE_BIT_MASK) && (status & LEP_I2C_STATUS_BOOT_STATUS_BIT_MASK);
            stateErrors = stateErrors || _lastI2CError || _lastLepResult;
            
            if (stateErrors) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                Serial.println("  LeptonFLiR::tryReadNextFrame Errors reading state encountered. Aborting.");
#endif
                _isReadingNextFrame = false;
                return false;
            }

            if (!cameraBooted) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                Serial.println("  LeptonFLiR::tryReadNextFrame Camera has not yet booted. Aborting.");
#endif
                _isReadingNextFrame = false;
                return false;
            }

            if (telemetryEnabled && !_telemetryData) {
                _telemetryData = (byte *)malloc(LEPFLIR_SPI_FRAME_PACKET_SIZE);

                if (_telemetryData)
                    _telemetryData[0] = _telemetryData[1] = 0xFF; // initialize as discard packet
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                if (!_telemetryData)
                    Serial.println("  LeptonFLiR::tryReadNextFrame Failure allocating telemetryData.");
#endif
            }
            else if (!telemetryEnabled && _telemetryData) {
                free(_telemetryData);
                _telemetryData = NULL;
            }
        }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.print("  LeptonFLiR::tryReadNextFrame AGC-8bit: ");
        Serial.print(agc8Enabled ? "enabled" : "disabled");
        Serial.print(", Telemetry: ");
        if (_telemetryData) {
            Serial.print("enabled, Location: ");
            Serial.println(telemetryLocation == LEP_TELEMETRY_LOCATION_HEADER ? "header" : "footer");
        }
        else
            Serial.println("disabled");
#endif

        uint16_t *spiFrame = NULL;
        uint_fast8_t imgRows = getImageHeight();
        uint_fast8_t currImgRow = 0;
        uint_fast8_t spiRows = getSPIFrameLines();
        uint_fast8_t currSpiRow = 0;
        uint_fast8_t teleRows = (_telemetryData ? 4 : 0);
        uint_fast8_t currTeleRow = 0;
        uint_fast8_t currReadRow = 0;        
        uint_fast8_t framesSkipped = 0;
        uint_fast8_t currRow = 0;
        bool skipFrame = false;
        bool spiPacketRead = false;

        SPI.beginTransaction(_spiSettings);

        _csEnableFunc(_spiCSPin);
        _csDisableFunc(_spiCSPin);
        delayTimeout(185);

        _csEnableFunc(_spiCSPin);
        
        while (currImgRow < imgRows || currTeleRow < teleRows) {
            if (!spiPacketRead) {
                spiFrame = getSPIFrameDataRow(currSpiRow);

                SPI_transfer16(spiFrame, LEPFLIR_SPI_FRAME_PACKET_SIZE16);
                
                skipFrame = ((spiFrame[0] & 0x0F00) == 0x0F00);
                currRow = (spiFrame[0] & 0x00FF);
            }
            else
                spiPacketRead = false;

            if (!skipFrame && currRow == currReadRow && (
                ((!teleRows || telemetryLocation == LEP_TELEMETRY_LOCATION_FOOTER) && currRow < 60) ||
                (telemetryLocation == LEP_TELEMETRY_LOCATION_HEADER && currReadRow >= teleRows))) { // Image packet
#if defined(LEPFLIR_ENABLE_DEBUG_OUTPUT) && defined(LEPFLIR_ENABLE_FRAME_PACKET_DEBUG_OUTPUT)
                Serial.println("    LeptonFLiR::tryReadNextFrame VoSPI Image Packet:");
                Serial.print("      ");  printSPIFrame(spiFrame);
#endif

                ++currReadRow; ++currSpiRow;
            }
            else if (!skipFrame && currRow == currReadRow && teleRows &&
                ((telemetryLocation == LEP_TELEMETRY_LOCATION_HEADER && currReadRow < teleRows) ||
                 (telemetryLocation == LEP_TELEMETRY_LOCATION_FOOTER && currReadRow >= 60))) { // Telemetry packet
                if (currTeleRow == 0)
                    memcpy(_telemetryData, spiFrame, LEPFLIR_SPI_FRAME_PACKET_SIZE);

#if defined(LEPFLIR_ENABLE_DEBUG_OUTPUT) && defined(LEPFLIR_ENABLE_FRAME_PACKET_DEBUG_OUTPUT)
                Serial.print("    LeptonFLiR::tryReadNextFrame VoSPI Telemetry(");
                Serial.print((char)('A' + currTeleRow));
                Serial.println(") Packet:");
                Serial.print("      ");  printSPIFrame(spiFrame);
#endif

                ++currReadRow; ++currTeleRow;
            }
            else if (!skipFrame && currRow < currReadRow) { // Ignore packet
#if defined(LEPFLIR_ENABLE_DEBUG_OUTPUT) && defined(LEPFLIR_ENABLE_FRAME_PACKET_DEBUG_OUTPUT)
                Serial.println("    LeptonFLiR::tryReadNextFrame VoSPI Ignore Packet:");
                Serial.print("      ");  printSPIFrame(spiFrame);
#endif
            }
            else { // Discard packet
#if defined(LEPFLIR_ENABLE_DEBUG_OUTPUT) && defined(LEPFLIR_ENABLE_FRAME_PACKET_DEBUG_OUTPUT)
                Serial.println("    LeptonFLiR::tryReadNextFrame VoSPI Discard Packet:");
                Serial.print("      ");  printSPIFrame(spiFrame);
#endif

                if (skipFrame && (currReadRow || framesSkipped)) {
                    _csDisableFunc(_spiCSPin);
                    delayTimeout(185);
                    _csEnableFunc(_spiCSPin);
                }

                uint_fast8_t triesLeft = 120;
                spiPacketRead = true;
                
                while (triesLeft > 0) {
                    SPI_transfer16(spiFrame, LEPFLIR_SPI_FRAME_PACKET_SIZE16);
                    
                    skipFrame = ((spiFrame[0] & 0x0F00) == 0x0F00);
                    currRow = (spiFrame[0] & 0x00FF);

                    if (!skipFrame) {
                        if (currRow == currReadRow) { // Reestablished sync at position we're next expecting
                            break;
                        }
                        else if (currRow == 0) { // Reestablished sync at next frame position
                            if ((currReadRow || framesSkipped) && ++framesSkipped >= 5) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                                Serial.println("  LeptonFLiR::tryReadNextFrame Maximum frame skip reached. Aborting.");
#endif

                                _csDisableFunc(_spiCSPin);
                                SPI.endTransaction();
                                _isReadingNextFrame = false;
                                return false;
                            }
                            else {
                                currReadRow = currImgRow = currSpiRow = currTeleRow = 0;

                                uint16_t* prevSPIFrame = spiFrame;
                                spiFrame = getSPIFrameDataRow(currSpiRow);
                                if (spiFrame != prevSPIFrame)
                                    memcpy(spiFrame, prevSPIFrame, LEPFLIR_SPI_FRAME_PACKET_SIZE);

                                break;
                            }
                        }
                    }

#if defined(LEPFLIR_ENABLE_DEBUG_OUTPUT) && defined(LEPFLIR_ENABLE_FRAME_PACKET_DEBUG_OUTPUT)
                    Serial.println("    LeptonFLiR::tryReadNextFrame VoSPI Discard Retry Packet:");
                    Serial.print("      ");  printSPIFrame(spiFrame);
#endif

                    --triesLeft;
                }

                if (triesLeft == 0) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                    Serial.println("  LeptonFLiR::tryReadNextFrame Maximum resync retries reached. Aborting.");
#endif

                    _csDisableFunc(_spiCSPin);
                    SPI.endTransaction();
                    _isReadingNextFrame = false;
                    return false;
                }
            }

            // Write out to frame
            if (currSpiRow == spiRows) {
                if (_imageMode == LeptonFLiR_ImageMode_80x60_16bpp) {
                    memcpy(_getImageDataRow(currImgRow), getSPIFrameDataRow(0) + 2, LEPFLIR_SPI_FRAME_PACKET_SIZE - 4);
                }
                else if (_imageMode == LeptonFLiR_ImageMode_80x60_8bpp && agc8Enabled) {
                    byte *pxlData = _getImageDataRow(currImgRow);
                    spiFrame = getSPIFrameDataRow(0) + 2;
                    uint_fast8_t size = LEPFLIR_SPI_FRAME_PACKET_SIZE16 - 2;
                    while (size--)
                        *pxlData++ = (byte)constrain(*spiFrame++, 0, 0x00FF);
                }
                else {
                    spiFrame = getSPIFrameDataRow(0) + 2;
                    byte *pxlData = _getImageDataRow(currImgRow);

                    uint_fast8_t imgWidth = getImageWidth();
                    uint_fast8_t imgBpp = getImageBpp();
                    uint_fast8_t spiPitch16 = roundUpVal16(LEPFLIR_SPI_FRAME_PACKET_SIZE) / 2;

                    uint_fast32_t divisor = (spiRows * spiRows) * (!agc8Enabled && imgBpp == 1 ? 64 : 1);
                    uint_fast32_t clamp = (!agc8Enabled && imgBpp == 2 ? 0x3FFF : 0x00FF);

                    while (imgWidth-- > 0) {
                        uint_fast32_t total = 0;

                        uint_fast8_t y = spiRows;
                        uint16_t *spiYFrame = spiFrame;
                        while (y-- > 0) {

                            uint_fast8_t x = spiRows;
                            uint16_t *spiXFrame = spiYFrame;
                            while (x-- > 0)
                                total += *spiXFrame++;

                            spiYFrame += spiPitch16;
                        }

                        total /= divisor;

                        if (imgBpp == 2)
                            *((uint16_t *)pxlData) = (uint16_t)constrain(total, 0, clamp);
                        else
                            *((byte *)pxlData) = (byte)constrain(total, 0, clamp);
                        pxlData += imgBpp;
                        spiFrame += spiRows;
                    }
                }

                ++currImgRow; currSpiRow = 0;
            }
        }

        SPI.endTransaction();

        _isReadingNextFrame = false;
    }

    return true;
}
