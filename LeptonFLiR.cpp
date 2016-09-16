/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016 NachtRaveVL      <nachtravevl@gmail.com>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    This permission notice shall be included in all copies or
    substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.

    Lepton-FLiR-Arduino - Version 0.9.91
*/

#include "LeptonFLiR.h"
#if (defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)) && !defined(LEPFLIR_DISABLE_SCHEDULER)
#include "Scheduler.h"
#define LEPFLIR_USE_SCHEDULER           1
#endif

#define LEPFLIR_GEN_CMD_TIMEOUT         5000        // Timeout for commands to be processed
#define LEPFLIR_SPI_MAX_SPEED           20000000    // Maximum SPI speed for FLiR module
#define LEPFLIR_SPI_MIN_SPEED           2200000     // Minimum SPI speed for FLiR module
#define LEPFLIR_SPI_FRAME_PACKET_SIZE           164 // 2B ID + 2B CRC + 160B for 80x1 14bpp/8bppAGC thermal image data or telemetry data
#define LEPFLIR_SPI_FRAME_PACKET_SIZE16         82

#ifndef LEPFLIR_DISABLE_ALIGNED_MALLOC
static inline int roundUpVal16(int val) { return ((val + 15) & -16); }
static inline byte *roundUpPtr16(byte *ptr) { return ptr ? (byte *)(((uintptr_t)ptr + 15) & -16) : NULL; }
static inline byte *roundUpMalloc16(int size) { return (byte *)malloc((size_t)(size + 15)); }
static inline byte *roundUpSpiFrame16(byte *spiFrame) { return spiFrame ? roundUpPtr16(spiFrame) + 16 - 4 : NULL; }
#else
static inline int roundUpVal16(int val) { return val; }
static inline byte *roundUpPtr16(byte *ptr) { return ptr; }
static inline byte *roundUpMalloc16(int size) { return (byte *)malloc((size_t)size); }
static inline byte *roundUpSpiFrame16(byte *spiFrame) { return spiFrame; }
#endif

#ifndef digitalWriteFast
static void csEnableFuncDef(byte pin) { digitalWrite(pin, LOW); }
static void csDisableFuncDef(byte pin) { digitalWrite(pin, HIGH); }
#else
static void csEnableFuncDef(byte pin) { digitalWriteFast(pin, LOW); }
static void csDisableFuncDef(byte pin) { digitalWriteFast(pin, HIGH); }
#endif

#ifndef LEPFLIR_USE_SOFTWARE_I2C
LeptonFLiR::LeptonFLiR(TwoWire& i2cWire, byte spiCSPin) {
    _i2cWire = &i2cWire;
#else
LeptonFLiR::LeptonFLiR(byte spiCSPin) {
#endif
    _spiCSPin = spiCSPin;
    _spiSettings = SPISettings(LEPFLIR_SPI_MAX_SPEED, MSBFIRST, SPI_MODE3);
    _storageMode = LeptonFLiR_ImageStorageMode_Count;
    _csEnableFunc = csEnableFuncDef;
    _csDisableFunc = csDisableFuncDef;
    _imageData = _spiFrameData = _telemetryData = NULL;
    _isReadingNextFrame = false;
    _lastI2CError = _lastLepResult = 0;
}

LeptonFLiR::~LeptonFLiR() {
    if (_imageData) free(_imageData);
    if (_spiFrameData) free(_spiFrameData);
    if (_telemetryData) free(_telemetryData);
}

void LeptonFLiR::init(LeptonFLiR_ImageStorageMode storageMode, LeptonFLiR_TemperatureMode tempMode) {
    _storageMode = (LeptonFLiR_ImageStorageMode)constrain((int)storageMode, 0, (int)LeptonFLiR_ImageStorageMode_Count - 1);
    _tempMode = (LeptonFLiR_TemperatureMode)constrain((int)tempMode, 0, (int)LeptonFLiR_TemperatureMode_Count - 1);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("LeptonFLiR::init spiCSPin: ");
    Serial.print(_spiCSPin);
    Serial.print(", storageMode: ");
    Serial.print(_storageMode);
    Serial.print(", tempMode: ");
    Serial.println(_tempMode);
#endif

    pinMode(_spiCSPin, OUTPUT);
    _csDisableFunc(_spiCSPin);

    _imageData = roundUpMalloc16(getImageTotalBytes());
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    if (!_imageData)
        Serial.println("  LeptonFLiR::init Failure allocating imageData.");
#endif

    _spiFrameData = roundUpMalloc16(getSPIFrameTotalBytes());
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    if (!_spiFrameData)
        Serial.println("  LeptonFLiR::init Failure allocating spiFrameData.");
#endif

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    int mallocOffset = 0;
#ifndef LEPFLIR_DISABLE_ALIGNED_MALLOC
    mallocOffset = 15;
#endif
    Serial.print("  LeptonFLiR::init imageData: ");
    Serial.print(_imageData ? getImageTotalBytes() + mallocOffset : 0);
    Serial.print("B, spiFrameData: ");
    Serial.print(_spiFrameData ? getSPIFrameTotalBytes() + mallocOffset : 0);
    Serial.print("B, total: ");
    Serial.print((_imageData ? getImageTotalBytes() + mallocOffset : 0) + (_spiFrameData ? getSPIFrameTotalBytes() + mallocOffset : 0));
    Serial.println("B");
    Serial.print("  LeptonFLiR::init SPIPortSpeed: ");
    for (int divisor = 2; divisor <= 128; divisor *= 2) {
        if (F_CPU / (float)divisor <= LEPFLIR_SPI_MAX_SPEED + 0.00001f || divisor == 128) {
            Serial.print(roundf((F_CPU / (float)divisor) / 1000.0f) / 1000.0f);
            Serial.print("MHz (SPI_CLOCK_DIV");
            Serial.print(divisor);
            Serial.print(")");
            break;
        }
    }
    if (F_CPU / 2.0f < LEPFLIR_SPI_MIN_SPEED - 0.00001f)
        Serial.println(" <speed too low>");
    else if (F_CPU / 128.0f > LEPFLIR_SPI_MAX_SPEED + 0.00001f)
        Serial.println(" <speed too high>");
    else
        Serial.println("");
#endif
}

byte LeptonFLiR::getChipSelectPin() {
    return _spiCSPin;
}

LeptonFLiR_ImageStorageMode LeptonFLiR::getImageStorageMode() {
    return _storageMode;
}

LeptonFLiR_TemperatureMode LeptonFLiR::getTemperatureMode() {
    return _tempMode;
}

void LeptonFLiR::setFastCSFuncs(digitalWriteFunc csEnableFunc, digitalWriteFunc csDisableFunc) {
    _csEnableFunc = csEnableFunc ? : csEnableFuncDef;
    _csDisableFunc = csDisableFunc ? : csDisableFuncDef;
}

int LeptonFLiR::getImageWidth() {
    switch (_storageMode) {
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
            return 80;
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
            return 40;
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            return 20;
        default:
            return 0;
    }
}

int LeptonFLiR::getImageHeight() {
    switch (_storageMode) {
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
            return 60;
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
            return 30;
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            return 15;
        default:
            return 0;
    }
}

int LeptonFLiR::getImageBpp() {
    switch (_storageMode) {
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
            return 2;
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            return 1;
        default:
            return 0;
    }
}

int LeptonFLiR::getImagePitch() {
    switch (_storageMode) {
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
            return roundUpVal16(80 * 2);
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
            return roundUpVal16(80 * 1);
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
            return roundUpVal16(40 * 2);
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
            return roundUpVal16(40 * 1);
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
            return roundUpVal16(20 * 2);
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            return roundUpVal16(20 * 1);
        default:
            return 0;
    }
}

int LeptonFLiR::getImageTotalBytes() {
    return ((getImageHeight() - 1) * getImagePitch()) + (getImageWidth() * getImageBpp());
}

byte *LeptonFLiR::getImageData() {
    return !_isReadingNextFrame && _imageData ? roundUpPtr16(_imageData) : NULL;
}

byte *LeptonFLiR::getImageDataRow(int row) {
    return !_isReadingNextFrame && _imageData ? (roundUpPtr16(_imageData) + (row * getImagePitch())) : NULL;
}

byte *LeptonFLiR::_getImageDataRow(int row) {
    return _imageData ? roundUpPtr16(_imageData) + (getImagePitch() * row) : NULL;
}

uint16_t LeptonFLiR::getImageDataRowCol(int row, int col) {
    if (_isReadingNextFrame || !_imageData) return 0;
    byte *imageData = roundUpPtr16(_imageData) + (row * getImagePitch()) + (col * getImageBpp());
    return getImageBpp() == 2 ? *((uint16_t *)imageData) : (uint16_t)(*imageData);
}

byte *LeptonFLiR::getTelemetryData() {
    return !_isReadingNextFrame && _telemetryData && !(*((uint16_t *)_telemetryData) & 0x0F00 == 0x0F00) ? _telemetryData : NULL;
}

void LeptonFLiR::getTelemetryData(TelemetryData *telemetry) {
    if (_isReadingNextFrame || !_telemetryData || !telemetry) return;
    uint16_t *telemetryData = (uint16_t *)&_telemetryData[4];

    telemetry->revisionMajor = lowByte(telemetryData[0]);
    telemetry->revisionMinor = highByte(telemetryData[0]);

    telemetry->cameraUptime = ((uint32_t)telemetryData[1] << 16) | (uint32_t)telemetryData[2];

    telemetry->ffcDesired = telemetryData[4] & 0x0004;
    uint_fast8_t ffcState = (telemetryData[4] & 0x0018) >> 3;
    if (telemetry->revisionMajor >= 9 && ffcState >= 1)
        ffcState -= 1;
    telemetry->ffcState = (TelemetryData_FFCState)ffcState;
    telemetry->agcEnabled = telemetryData[4] & 0x0800;
    telemetry->shutdownImminent = telemetryData[3] & 0x0010;

    wordsToHexString(&telemetryData[5], 8, telemetry->serialNumber, 24);
    wordsToHexString(&telemetryData[13], 4, telemetry->softwareRevision, 12);

    telemetry->frameCounter = ((uint32_t)telemetryData[20] << 16) | (uint32_t)telemetryData[21];
    telemetry->frameMean = telemetryData[22];

    telemetry->fpaTemperature = kelvin100ToTemperature(telemetryData[24]);
    telemetry->housingTemperature = kelvin100ToTemperature(telemetryData[26]);

    telemetry->lastFFCTime = ((uint32_t)telemetryData[30] << 16) | (uint32_t)telemetryData[31];
    telemetry->fpaTempAtLastFFC = kelvin100ToTemperature(telemetryData[29]);
    telemetry->housingTempAtLastFFC = kelvin100ToTemperature(telemetryData[32]);

    telemetry->agcRegion.startRow = telemetryData[34];
    telemetry->agcRegion.startCol = telemetryData[35];
    telemetry->agcRegion.endCol = telemetryData[36];
    telemetry->agcRegion.endRow = telemetryData[37];

    telemetry->agcClipHigh = telemetryData[38];
    telemetry->agcClipLow = telemetryData[39];

    telemetry->log2FFCFrames = telemetryData[74];
}

uint32_t LeptonFLiR::getTelemetryFrameCounter() {
    if (_isReadingNextFrame || !_telemetryData) return 0;
    uint16_t *telemetryData = (uint16_t *)&_telemetryData[4];

    return ((uint32_t)telemetryData[20] << 16) | (uint32_t)telemetryData[21];
}

bool LeptonFLiR::getShouldRunFFCNormalization() {
    if (_isReadingNextFrame || !_telemetryData) return false;
    uint16_t *telemetryData = (uint16_t *)&_telemetryData[4];

    uint_fast8_t ffcState = (telemetryData[4] & 0x0018) >> 3;
    if (lowByte(telemetryData[0]) >= 9 && ffcState >= 1)
        ffcState -= 1;

    return (telemetryData[4] & 0x0004) && ffcState != (uint_fast8_t)TelemetryData_FFCState_InProgress;
}

int LeptonFLiR::getSPIFrameLines() {
    switch (_storageMode) {
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
            return 1;
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
            return 2;
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            return 4;
        default:
            return 0;
    }
}

int LeptonFLiR::getSPIFrameTotalBytes() {
    return getSPIFrameLines() * roundUpVal16(LEPFLIR_SPI_FRAME_PACKET_SIZE);
}

uint16_t *LeptonFLiR::getSPIFrameDataRow(int row) {
    return (uint16_t *)(roundUpSpiFrame16(_spiFrameData) + (row * roundUpVal16(LEPFLIR_SPI_FRAME_PACKET_SIZE)));
}

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

static void printSPIFrame(uint16_t *spiFrame) {
    Serial.print("ID: 0x");
    Serial.print(spiFrame[0], HEX);
    Serial.print(" CRC: 0x");
    Serial.print(spiFrame[1], HEX);
    Serial.print(" Data: ");
    for (int i = 0; i < 5; ++i) {
        Serial.print(i > 0 ? "-0x" : "0x");
        Serial.print(spiFrame[i + 2], HEX);
    }
    Serial.print("...");
    for (int i = 75; i < 80; ++i) {
        Serial.print(i > 75 ? "-0x" : "0x");
        Serial.print(spiFrame[i + 2], HEX);
    }
    Serial.println("");
}

#endif

static void delayTimeout(int timeout) {
    unsigned long endTime = millis() + (unsigned long)timeout;

    while (millis() < endTime) {
#ifdef LEPFLIR_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif
    }
}

static void SPI_transfer16(uint16_t *buffer, int count) {
    while (count-- > 0)
        *buffer++ = SPI.transfer16(0x0000);
}

static void SPI_ignore16(int count) {
    while (count-- > 0)
        SPI.transfer16(0x0000);
}

//#define LEPFLIR_ENABLE_FRAME_PACKET_DEBUG_OUTPUT    1

bool LeptonFLiR::readNextFrame() {
    if (!_isReadingNextFrame) {
        _isReadingNextFrame = true;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.println("LeptonFLiR::readNextFrame");
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
                Serial.println("  LeptonFLiR::readNextFrame Errors reading state encountered. Aborting.");
#endif
                _isReadingNextFrame = false;
                return false;
            }

            if (!cameraBooted) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                Serial.println("  LeptonFLiR::readNextFrame Camera has not yet booted. Aborting.");
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
                    Serial.println("  LeptonFLiR::readNextFrame Failure allocating telemetryData.");
#endif
            }
            else if (!telemetryEnabled && _telemetryData) {
                free(_telemetryData);
                _telemetryData = NULL;
            }
        }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.print("  LeptonFLiR::readNextFrame AGC-8bit: ");
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
                Serial.println("    LeptonFLiR::readNextFrame VoSPI Image Packet:");
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
                Serial.print("    LeptonFLiR::readNextFrame VoSPI Telemetry(");
                Serial.print((char)('A' + currTeleRow));
                Serial.println(") Packet:");
                Serial.print("      ");  printSPIFrame(spiFrame);
#endif

                ++currReadRow; ++currTeleRow;
            }
            else if (!skipFrame && currRow < currReadRow) { // Ignore packet
#if defined(LEPFLIR_ENABLE_DEBUG_OUTPUT) && defined(LEPFLIR_ENABLE_FRAME_PACKET_DEBUG_OUTPUT)
                Serial.println("    LeptonFLiR::readNextFrame VoSPI Ignore Packet:");
                Serial.print("      ");  printSPIFrame(spiFrame);
#endif
            }
            else { // Discard packet
#if defined(LEPFLIR_ENABLE_DEBUG_OUTPUT) && defined(LEPFLIR_ENABLE_FRAME_PACKET_DEBUG_OUTPUT)
                Serial.println("    LeptonFLiR::readNextFrame VoSPI Discard Packet:");
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
                                Serial.println("  LeptonFLiR::readNextFrame Maximum frame skip reached. Aborting.");
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
                    Serial.println("    LeptonFLiR::readNextFrame VoSPI Discard Retry Packet:");
                    Serial.print("      ");  printSPIFrame(spiFrame);
#endif

                    --triesLeft;
                }

                if (triesLeft == 0) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                    Serial.println("  LeptonFLiR::readNextFrame Maximum resync retries reached. Aborting.");
#endif

                    _csDisableFunc(_spiCSPin);
                    SPI.endTransaction();
                    _isReadingNextFrame = false;
                    return false;
                }
            }

            // Write out to frame
            if (currSpiRow == spiRows) {
                if (_storageMode == LeptonFLiR_ImageStorageMode_80x60_16bpp) {
                    memcpy(_getImageDataRow(currImgRow), getSPIFrameDataRow(0) + 2, LEPFLIR_SPI_FRAME_PACKET_SIZE - 4);
                }
                else if (_storageMode == LeptonFLiR_ImageStorageMode_80x60_8bpp && agc8Enabled) {
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

void LeptonFLiR::agc_setAGCEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setAGCEnabled");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::agc_getAGCEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getAGCEnabled");
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::agc_setAGCPolicy(LEP_AGC_POLICY policy) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setAGCPolicy");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_POLICY, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)policy);
}

LEP_AGC_POLICY LeptonFLiR::agc_getAGCPolicy() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getAGCPolicy");
#endif

    uint32_t policy;
    receiveCommand(cmdCode(LEP_CID_AGC_POLICY, LEP_I2C_COMMAND_TYPE_GET), &policy);
    return (LEP_AGC_POLICY)policy;
}

void LeptonFLiR::agc_setHEQScaleFactor(LEP_AGC_HEQ_SCALE_FACTOR factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHEQScaleFactor");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_SCALE_FACTOR, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)factor);
}

LEP_AGC_HEQ_SCALE_FACTOR LeptonFLiR::agc_getHEQScaleFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHEQScaleFactor");
#endif

    uint32_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_SCALE_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return (LEP_AGC_HEQ_SCALE_FACTOR)factor;
}

void LeptonFLiR::agc_setAGCCalcEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setAGCCalcEnabled");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_CALC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::agc_getAGCCalcEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getAGCCalcEnabled");
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_AGC_CALC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::sys_getCameraStatus(LEP_SYS_CAM_STATUS *status) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getCameraStatus");
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_CAM_STATUS, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)status, sizeof(LEP_SYS_CAM_STATUS) / 2);
}

LEP_SYS_CAM_STATUS_STATES LeptonFLiR::sys_getCameraStatus() {
    LEP_SYS_CAM_STATUS camStatus;
    sys_getCameraStatus(&camStatus);
    return (LEP_SYS_CAM_STATUS_STATES)camStatus.camStatus;
}

void LeptonFLiR::sys_getFlirSerialNumber(char *buffer, int maxLength) {
    if (!buffer || maxLength < 16) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getFlirSerialNumber");
#endif

    uint16_t innerBuffer[4];
    receiveCommand(cmdCode(LEP_CID_SYS_FLIR_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), innerBuffer, 4);
    wordsToHexString(innerBuffer, 4, buffer, maxLength);
}

void LeptonFLiR::sys_getCustomerSerialNumber(char *buffer, int maxLength) {
    if (!buffer || maxLength < 64) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getCustomerSerialNumber");
#endif

    uint16_t innerBuffer[16];
    receiveCommand(cmdCode(LEP_CID_SYS_CUST_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), innerBuffer, 16);
    wordsToHexString(innerBuffer, 16, buffer, maxLength);
}

uint32_t LeptonFLiR::sys_getCameraUptime() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getCameraUptime");
#endif

    uint32_t uptime;
    receiveCommand(cmdCode(LEP_CID_SYS_CAM_UPTIME, LEP_I2C_COMMAND_TYPE_GET), &uptime);
    return uptime;
}

float LeptonFLiR::sys_getAuxTemperature() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getAuxTemperature");
#endif

    uint16_t kelvin100;
    receiveCommand(cmdCode(LEP_CID_SYS_AUX_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), &kelvin100);
    return kelvin100ToTemperature(kelvin100);
}

float LeptonFLiR::sys_getFPATemperature() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getFPATemperature");
#endif

    uint16_t kelvin100;
    receiveCommand(cmdCode(LEP_CID_SYS_FPA_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), &kelvin100);
    return kelvin100ToTemperature(kelvin100);
}

void LeptonFLiR::sys_setTelemetryEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setTelemetryEnabled");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);

    if (!_lastI2CError && !_lastLepResult) {
        if (enabled && !_telemetryData) {
            _telemetryData = (byte *)malloc(LEPFLIR_SPI_FRAME_PACKET_SIZE);

            if (_telemetryData)
                _telemetryData[0] = _telemetryData[1] = 0xFF; // initialize as discard packet
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
            if (!_telemetryData)
                Serial.println("  LeptonFLiR::sys_setTelemetryEnabled Failure allocating telemetryData.");
#endif
        }
        else if (!enabled && _telemetryData) {
            free(_telemetryData);
            _telemetryData = NULL;
        }
    }
}

bool LeptonFLiR::sys_getTelemetryEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getTelemetryEnabled");
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);

    if (!_lastI2CError && !_lastLepResult) {
        if (enabled && !_telemetryData) {
            _telemetryData = (byte *)malloc(LEPFLIR_SPI_FRAME_PACKET_SIZE);

            if (_telemetryData)
                _telemetryData[0] = _telemetryData[1] = 0xFF; // initialize as discard packet
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
            if (!_telemetryData)
                Serial.println("  LeptonFLiR::sys_getTelemetryEnabled Failure allocating telemetryData.");
#endif
        }
        else if (!enabled && _telemetryData) {
            free(_telemetryData);
            _telemetryData = NULL;
        }
    }

    return enabled;
}

void LeptonFLiR::sys_runFFCNormalization() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_runFFCNormalization");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_RUN_FFC, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::vid_setPolarity(LEP_VID_POLARITY polarity) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setPolarity");
#endif

    sendCommand(cmdCode(LEP_CID_VID_POLARITY_SELECT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)polarity);
}

LEP_VID_POLARITY LeptonFLiR::vid_getPolarity() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getPolarity");
#endif

    uint32_t polarity;
    receiveCommand(cmdCode(LEP_CID_VID_POLARITY_SELECT, LEP_I2C_COMMAND_TYPE_GET), &polarity);
    return (LEP_VID_POLARITY)polarity;
}

void LeptonFLiR::vid_setPseudoColorLUT(LEP_VID_PCOLOR_LUT table) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setPseudoColorLUT");
#endif

    sendCommand(cmdCode(LEP_CID_VID_LUT_SELECT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)table);
}

LEP_VID_PCOLOR_LUT LeptonFLiR::vid_getPseudoColorLUT() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getPseudoColorLUT");
#endif

    uint32_t table;
    receiveCommand(cmdCode(LEP_CID_VID_LUT_SELECT, LEP_I2C_COMMAND_TYPE_GET), &table);
    return (LEP_VID_PCOLOR_LUT)table;
}

void LeptonFLiR::vid_setFocusCalcEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setFocusCalcEnabled");
#endif

    sendCommand(cmdCode(LEP_CID_VID_FOCUS_CALC_ENABLE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::vid_getFocusCalcEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getFocusCalcEnabled");
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_CALC_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::vid_setFreezeEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setFreezeEnabled");
#endif

    sendCommand(cmdCode(LEP_CID_VID_FREEZE_ENABLE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::vid_getFreezeEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getFreezeEnabled");
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_VID_FREEZE_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

#ifndef LEPFLIR_EXCLUDE_EXT_I2C_FUNCS

void LeptonFLiR::agc_setHistogramRegion(LEP_AGC_HISTOGRAM_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHistogramRegion");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_ROI, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)region, sizeof(LEP_AGC_HISTOGRAM_ROI) / 2);
}

void LeptonFLiR::agc_getHistogramRegion(LEP_AGC_HISTOGRAM_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHistogramRegion");
#endif

    receiveCommand(cmdCode(LEP_CID_AGC_ROI, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)region, sizeof(LEP_AGC_HISTOGRAM_ROI) / 2);
}

void LeptonFLiR::agc_getHistogramStatistics(LEP_AGC_HISTOGRAM_STATISTICS *statistics) {
    if (!statistics) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHistogramStatistics");
#endif

    receiveCommand(cmdCode(LEP_CID_AGC_STATISTICS, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)statistics, sizeof(LEP_AGC_HISTOGRAM_STATISTICS) / 2);
}

void LeptonFLiR::agc_setHistogramClipPercent(uint16_t percent) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHistogramClipPercent");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_CLIP_PERCENT, LEP_I2C_COMMAND_TYPE_SET), percent);
}

uint16_t LeptonFLiR::agc_getHistogramClipPercent() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHistogramClipPercent");
#endif

    uint16_t percent;
    receiveCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_CLIP_PERCENT, LEP_I2C_COMMAND_TYPE_GET), &percent);
    return percent;
}

void LeptonFLiR::agc_setHistogramTailSize(uint16_t size) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHistogramTailSize");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_TAIL_SIZE, LEP_I2C_COMMAND_TYPE_SET), size);
}

uint16_t LeptonFLiR::agc_getHistogramTailSize() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHistogramTailSize");
#endif

    uint16_t size;
    receiveCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_TAIL_SIZE, LEP_I2C_COMMAND_TYPE_GET), &size);
    return size;
}

void LeptonFLiR::agc_setLinearMaxGain(uint16_t gain) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setLinearMaxGain");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_LINEAR_MAX_GAIN, LEP_I2C_COMMAND_TYPE_SET), gain);
}

uint16_t LeptonFLiR::agc_getLinearMaxGain() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getLinearMaxGain");
#endif

    uint16_t gain;
    receiveCommand(cmdCode(LEP_CID_AGC_LINEAR_MAX_GAIN, LEP_I2C_COMMAND_TYPE_GET), &gain);
    return gain;
}

void LeptonFLiR::agc_setLinearMidpoint(uint16_t midpoint) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setLinearMidpoint");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_LINEAR_MIDPOINT, LEP_I2C_COMMAND_TYPE_SET), midpoint);
}

uint16_t LeptonFLiR::agc_getLinearMidpoint() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getLinearMidpoint");
#endif

    uint16_t midpoint;
    receiveCommand(cmdCode(LEP_CID_AGC_LINEAR_MIDPOINT, LEP_I2C_COMMAND_TYPE_GET), &midpoint);
    return midpoint;
}

void LeptonFLiR::agc_setLinearDampeningFactor(uint16_t factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setLinearDampeningFactor");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_LINEAR_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_SET), factor);
}

uint16_t LeptonFLiR::agc_getLinearDampeningFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getLinearDampeningFactor");
#endif

    uint16_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_LINEAR_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return factor;
}

void LeptonFLiR::agc_setHEQDampeningFactor(uint16_t factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHEQDampeningFactor");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_SET), factor);
}

uint16_t LeptonFLiR::agc_getHEQDampeningFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHEQDampeningFactor");
#endif

    uint16_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return factor;
}

void LeptonFLiR::agc_setHEQMaxGain(uint16_t gain) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHEQMaxGain");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_MAX_GAIN, LEP_I2C_COMMAND_TYPE_SET), gain);
}

uint16_t LeptonFLiR::agc_getHEQMaxGain() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHEQMaxGain");
#endif

    uint16_t gain;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_MAX_GAIN, LEP_I2C_COMMAND_TYPE_GET), &gain);
    return gain;
}

void LeptonFLiR::agc_setHEQClipLimitHigh(uint16_t limit) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHEQClipLimitHigh");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_HIGH, LEP_I2C_COMMAND_TYPE_SET), limit);
}

uint16_t LeptonFLiR::agc_getHEQClipLimitHigh() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHEQClipLimitHigh");
#endif

    uint16_t limit;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_HIGH, LEP_I2C_COMMAND_TYPE_GET), &limit);
    return limit;
}

void LeptonFLiR::agc_setHEQClipLimitLow(uint16_t limit) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHEQClipLimitLow");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_LOW, LEP_I2C_COMMAND_TYPE_SET), limit);
}

uint16_t LeptonFLiR::agc_getHEQClipLimitLow() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHEQClipLimitLow");
#endif

    uint16_t limit;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_LOW, LEP_I2C_COMMAND_TYPE_GET), &limit);
    return limit;
}

void LeptonFLiR::agc_setHEQBinExtension(uint16_t extension) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHEQBinExtension");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_BIN_EXTENSION, LEP_I2C_COMMAND_TYPE_SET), extension);
}

uint16_t LeptonFLiR::agc_getHEQBinExtension() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHEQBinExtension");
#endif

    uint16_t extension;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_BIN_EXTENSION, LEP_I2C_COMMAND_TYPE_GET), &extension);
    return extension;
}

void LeptonFLiR::agc_setHEQMidpoint(uint16_t midpoint) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHEQMidpoint");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_MIDPOINT, LEP_I2C_COMMAND_TYPE_SET), midpoint);
}

uint16_t LeptonFLiR::agc_getHEQMidpoint() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHEQMidpoint");
#endif

    uint16_t midpoint;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_MIDPOINT, LEP_I2C_COMMAND_TYPE_GET), &midpoint);
    return midpoint;
}

void LeptonFLiR::agc_setHEQEmptyCounts(uint16_t counts) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHEQEmptyCounts");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_EMPTY_COUNTS, LEP_I2C_COMMAND_TYPE_SET), counts);
}

uint16_t LeptonFLiR::agc_getHEQEmptyCounts() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHEQEmptyCounts");
#endif

    uint16_t counts;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_EMPTY_COUNTS, LEP_I2C_COMMAND_TYPE_GET), &counts);
    return counts;
}

void LeptonFLiR::agc_setHEQNormalizationFactor(uint16_t factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_setHEQNormalizationFactor");
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_NORMALIZATION_FACTOR, LEP_I2C_COMMAND_TYPE_SET), factor);
}

uint16_t LeptonFLiR::agc_getHEQNormalizationFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::agc_getHEQNormalizationFactor");
#endif

    uint16_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_NORMALIZATION_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return factor;
}

void LeptonFLiR::sys_runPingCamera() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_runPingCamera");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_PING, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::sys_setTelemetryLocation(LEP_SYS_TELEMETRY_LOCATION location) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setTelemetryLocation");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_TELEMETRY_LOCATION, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)location);
}

LEP_SYS_TELEMETRY_LOCATION LeptonFLiR::sys_getTelemetryLocation() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getTelemetryLocation");
#endif

    uint32_t location;
    receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_LOCATION, LEP_I2C_COMMAND_TYPE_GET), &location);
    return (LEP_SYS_TELEMETRY_LOCATION)location;
}

void LeptonFLiR::sys_runFrameAveraging() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_runFrameAveraging");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_EXECTUE_FRAME_AVERAGE, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::sys_setNumFramesToAverage(LEP_SYS_FRAME_AVERAGE average) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setNumFramesToAverage");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)average);
}

LEP_SYS_FRAME_AVERAGE LeptonFLiR::sys_getNumFramesToAverage() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getNumFramesToAverage");
#endif

    uint32_t average;
    receiveCommand(cmdCode(LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE, LEP_I2C_COMMAND_TYPE_GET), &average);
    return (LEP_SYS_FRAME_AVERAGE)average;
}

void LeptonFLiR::sys_getSceneStatistics(LEP_SYS_SCENE_STATISTICS *statistics) {
    if (!statistics) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getSceneStatistics");
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_SCENE_STATISTICS, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)statistics, sizeof(LEP_SYS_SCENE_STATISTICS) / 2);
}

void LeptonFLiR::sys_setSceneRegion(LEP_SYS_SCENE_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setSceneRegion");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_SCENE_ROI, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)region, sizeof(LEP_SYS_SCENE_ROI) / 2);
}

void LeptonFLiR::sys_getSceneRegion(LEP_SYS_SCENE_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getSceneRegion");
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_SCENE_ROI, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)region, sizeof(LEP_SYS_SCENE_ROI) / 2);
}

uint16_t LeptonFLiR::sys_getThermalShutdownCount() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getThermalShutdownCount");
#endif

    uint16_t count;
    receiveCommand(cmdCode(LEP_CID_SYS_THERMAL_SHUTDOWN_COUNT, LEP_I2C_COMMAND_TYPE_GET), &count);
    return count;
}

void LeptonFLiR::sys_setShutterPosition(LEP_SYS_SHUTTER_POSITION position) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setShutterPosition");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_SHUTTER_POSITION, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)position);
}

LEP_SYS_SHUTTER_POSITION LeptonFLiR::sys_getShutterPosition() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getShutterPosition");
#endif

    uint32_t position;
    receiveCommand(cmdCode(LEP_CID_SYS_SHUTTER_POSITION, LEP_I2C_COMMAND_TYPE_GET), &position);
    return (LEP_SYS_SHUTTER_POSITION)position;
}

void LeptonFLiR::sys_setFFCShutterMode(LEP_SYS_FFC_SHUTTER_MODE *mode) {
    if (!mode) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setFFCShutterMode");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_FFC_SHUTTER_MODE, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)mode, sizeof(LEP_SYS_FFC_SHUTTER_MODE) / 2);
}

void LeptonFLiR::sys_getFFCShutterMode(LEP_SYS_FFC_SHUTTER_MODE *mode) {
    if (!mode) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getFFCShutterMode");
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_FFC_SHUTTER_MODE, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)mode, sizeof(LEP_SYS_FFC_SHUTTER_MODE) / 2);
}

LEP_SYS_FFC_STATUS LeptonFLiR::sys_getFFCNormalizationStatus() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getFFCNormalizationStatus");
#endif

    uint32_t status;
    receiveCommand(cmdCode(LEP_CID_SYS_FFC_STATUS, LEP_I2C_COMMAND_TYPE_GET), &status);
    return (LEP_SYS_FFC_STATUS)status;
}

void LeptonFLiR::vid_setUserColorLUT(LEP_VID_LUT_BUFFER *table) {
    if (!table) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setUserColorLUT");
#endif

    sendCommand(cmdCode(LEP_CID_VID_LUT_TRANSFER, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)table, sizeof(LEP_VID_LUT_BUFFER) / 2);
}

void LeptonFLiR::vid_getUserColorLUT(LEP_VID_LUT_BUFFER *table) {
    if (!table) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getUserColorLUT");
#endif

    receiveCommand(cmdCode(LEP_CID_VID_LUT_TRANSFER, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)table, sizeof(LEP_VID_LUT_BUFFER) / 2);
}

void LeptonFLiR::vid_setFocusRegion(LEP_VID_FOCUS_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setFocusRegion");
#endif

    sendCommand(cmdCode(LEP_CID_VID_FOCUS_ROI, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)region, sizeof(LEP_VID_FOCUS_ROI) / 2);
}

void LeptonFLiR::vid_getFocusRegion(LEP_VID_FOCUS_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getFocusRegion");
#endif

    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_ROI, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)region, sizeof(LEP_VID_FOCUS_ROI) / 2);
}

void LeptonFLiR::vid_setFocusThreshold(uint32_t threshold) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setFocusThreshold");
#endif

    sendCommand(cmdCode(LEP_CID_VID_FOCUS_THRESHOLD, LEP_I2C_COMMAND_TYPE_SET), threshold);
}

uint32_t LeptonFLiR::vid_getFocusThreshold() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getFocusThreshold");
#endif

    uint32_t threshold;
    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_THRESHOLD, LEP_I2C_COMMAND_TYPE_GET), &threshold);
    return threshold;
}

uint32_t LeptonFLiR::vid_getFocusMetric() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getFocusMetric");
#endif

    uint32_t metric;
    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_METRIC, LEP_I2C_COMMAND_TYPE_GET), &metric);
    return metric;
}

void LeptonFLiR::vid_setSceneBasedNUCEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setSceneBasedNUCEnabled");
#endif

    sendCommand(cmdCode(LEP_CID_VID_SBNUC_ENABLE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::vid_getSceneBasedNUCEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getSceneBasedNUCEnabled");
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_VID_SBNUC_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::vid_setGamma(uint32_t gamma) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setGamma");
#endif

    sendCommand(cmdCode(LEP_CID_VID_GAMMA_SELECT, LEP_I2C_COMMAND_TYPE_SET), gamma);
}

uint32_t LeptonFLiR::vid_getGamma() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getGamma");
#endif

    uint32_t gamma;
    receiveCommand(cmdCode(LEP_CID_VID_GAMMA_SELECT, LEP_I2C_COMMAND_TYPE_GET), &gamma);
    return gamma;
}

#endif

static inline void byteToHexString(byte value, char *buffer) {
    byte highNibble = value / 16;
    byte lowNibble = value % 16;
    if (highNibble < 10) buffer[0] = '0' + highNibble;
    else buffer[0] = 'A' + (highNibble - 10);
    if (lowNibble < 10) buffer[1] = '0' + lowNibble;
    else buffer[1] = 'A' + (lowNibble - 10);
}

void wordsToHexString(uint16_t *dataWords, int dataLength, char *buffer, int maxLength) {
    bool insertColons = maxLength >= (dataLength * 4) + (dataLength - 1);

    while (dataLength-- > 0 && maxLength > 3) {
        if (maxLength > 3) {
            byteToHexString(highByte(*dataWords), buffer);
            buffer += 2; maxLength -= 2;
            byteToHexString(lowByte(*dataWords), buffer);
            buffer += 2; maxLength -= 2;
            ++dataWords;
        }

        if (dataLength > 0 && insertColons && maxLength-- > 0)
            *buffer++ = ':';
    }

    if (maxLength-- > 0)
        *buffer++ = '\0';
}

float kelvin100ToCelsius(uint16_t kelvin100) {
    float kelvin = (kelvin100 / 100) + ((kelvin100 % 100) * 0.01f);
    return kelvin - 273.15f;
}

float kelvin100ToFahrenheit(uint16_t kelvin100) {
    float kelvin = (kelvin100 / 100) + ((kelvin100 % 100) * 0.01f);
    return roundf((((kelvin * 9.0f) / 5.0f) - 459.67f) * 100.0f) / 100.0f;
}

float kelvin100ToKelvin(uint16_t kelvin100) {
    return (kelvin100 / 100) + ((kelvin100 % 100) * 0.01f);
}

uint16_t celsiusToKelvin100(float celsius) {
    float kelvin = celsius + 273.15f;
    return (uint16_t)roundf(kelvin * 100.0f);
}

uint16_t fahrenheitToKelvin100(float fahrenheit) {
    float kelvin = ((fahrenheit + 459.67f) * 5.0f) / 9.0f;
    return (uint16_t)roundf(kelvin * 100.0f);
}

uint16_t kelvinToKelvin100(float kelvin) {
    return (uint16_t)roundf(kelvin * 100.0f);
}

float LeptonFLiR::kelvin100ToTemperature(uint16_t kelvin100) {
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            return kelvin100ToCelsius(kelvin100);
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            return kelvin100ToFahrenheit(kelvin100);
        case LeptonFLiR_TemperatureMode_Kelvin:
            return kelvin100ToKelvin(kelvin100);
        default:
            return 0;
    }
}

uint16_t LeptonFLiR::temperatureToKelvin100(float temperature) {
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            return celsiusToKelvin100(temperature);
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            return fahrenheitToKelvin100(temperature);
        case LeptonFLiR_TemperatureMode_Kelvin:
            return kelvinToKelvin100(temperature);
        default:
            return 0;
    }
}

const char *LeptonFLiR::getTemperatureSymbol() {
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            return "°C";
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            return "°F";
        case LeptonFLiR_TemperatureMode_Kelvin:
            return "°K";
        default:
            return "";
    }
}

byte LeptonFLiR::getLastI2CError() {
    return _lastI2CError;
}

LEP_RESULT LeptonFLiR::getLastLepResult() {
    return (LEP_RESULT)_lastLepResult;
}

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

static const char *textForI2CError(byte errorCode) {
    switch (errorCode) {
        case 0:
            return "Success";
        case 1:
            return "Data too long to fit in transmit buffer";
        case 2:
            return "Received NACK on transmit of address";
        case 3:
            return "Received NACK on transmit of data";
        default:
            return "Other error";
    }
}

static const char *textForLepResult(LEP_RESULT errorCode) {
    switch (errorCode) {
        case LEP_OK:
            return "LEP_OK Camera ok";
        case LEP_ERROR:
            return "LEP_ERROR Camera general error";
        case LEP_NOT_READY:
            return "LEP_NOT_READY Camera not ready error";
        case LEP_RANGE_ERROR:
            return "LEP_RANGE_ERROR Camera range error";
        case LEP_CHECKSUM_ERROR:
            return "LEP_CHECKSUM_ERROR Camera checksum error";
        case LEP_BAD_ARG_POINTER_ERROR:
            return "LEP_BAD_ARG_POINTER_ERROR Camera Bad argument  error";
        case LEP_DATA_SIZE_ERROR:
            return "LEP_DATA_SIZE_ERROR Camera byte count error";
        case LEP_UNDEFINED_FUNCTION_ERROR:
            return "LEP_UNDEFINED_FUNCTION_ERROR Camera undefined function error";
        case LEP_FUNCTION_NOT_SUPPORTED:
            return "LEP_FUNCTION_NOT_SUPPORTED Camera function not yet supported error";
        case LEP_OTP_WRITE_ERROR:
            return "LEP_OTP_WRITE_ERROR Camera OTP write error";
        case LEP_OTP_READ_ERROR:
            return "LEP_OTP_READ_ERROR Double bit error detected (uncorrectible)";
        case LEP_OTP_NOT_PROGRAMMED_ERROR:
            return "LEP_OTP_NOT_PROGRAMMED_ERROR Flag read as non-zero";
        case LEP_ERROR_I2C_BUS_NOT_READY:
            return "LEP_ERROR_I2C_BUS_NOT_READY I2C Bus Error - Bus Not Avaialble";
        case LEP_ERROR_I2C_BUFFER_OVERFLOW:
            return "LEP_ERROR_I2C_BUFFER_OVERFLOW I2C Bus Error - Buffer Overflow";
        case LEP_ERROR_I2C_ARBITRATION_LOST:
            return "LEP_ERROR_I2C_ARBITRATION_LOST I2C Bus Error - Bus Arbitration Lost";
        case LEP_ERROR_I2C_BUS_ERROR:
            return "LEP_ERROR_I2C_BUS_ERROR I2C Bus Error - General Bus Error";
        case LEP_ERROR_I2C_NACK_RECEIVED:
            return "LEP_ERROR_I2C_NACK_RECEIVED I2C Bus Error - NACK Received";
        case LEP_ERROR_I2C_FAIL:
            return "LEP_ERROR_I2C_FAIL I2C Bus Error - General Failure";
        case LEP_DIV_ZERO_ERROR:
            return "LEP_DIV_ZERO_ERROR Attempted div by zero";
        case LEP_COMM_PORT_NOT_OPEN:
            return "LEP_COMM_PORT_NOT_OPEN Comm port not open";
        case LEP_COMM_INVALID_PORT_ERROR:
            return "LEP_COMM_INVALID_PORT_ERROR Comm port no such port error";
        case LEP_COMM_RANGE_ERROR:
            return "LEP_COMM_RANGE_ERROR Comm port range error";
        case LEP_ERROR_CREATING_COMM:
            return "LEP_ERROR_CREATING_COMM Error creating comm";
        case LEP_ERROR_STARTING_COMM:
            return "LEP_ERROR_STARTING_COMM Error starting comm";
        case LEP_ERROR_CLOSING_COMM:
            return "LEP_ERROR_CLOSING_COMM Error closing comm";
        case LEP_COMM_CHECKSUM_ERROR:
            return "LEP_COMM_CHECKSUM_ERROR Comm checksum error";
        case LEP_COMM_NO_DEV:
            return "LEP_COMM_NO_DEV No comm device";
        case LEP_TIMEOUT_ERROR:
            return "LEP_TIMEOUT_ERROR Comm timeout error";
        case LEP_COMM_ERROR_WRITING_COMM:
            return "LEP_COMM_ERROR_WRITING_COMM Error writing comm";
        case LEP_COMM_ERROR_READING_COMM:
            return "LEP_COMM_ERROR_READING_COMM Error reading comm";
        case LEP_COMM_COUNT_ERROR:
            return "LEP_COMM_COUNT_ERROR Comm byte count error";
        case LEP_OPERATION_CANCELED:
            return "LEP_OPERATION_CANCELED Camera operation canceled";
        case LEP_UNDEFINED_ERROR_CODE:
            return "LEP_UNDEFINED_ERROR_CODE Undefined error";
        default:
            return "Other error";
    }
}

void LeptonFLiR::checkForErrors() {
    if (_lastI2CError) {
        Serial.print("  LeptonFLiR::checkErrors lastI2CError: ");
        Serial.print(_lastI2CError);
        Serial.print(": ");
        Serial.println(textForI2CError(getLastI2CError()));
    }
    if (_lastLepResult) {
        Serial.print("  LeptonFLiR::checkErrors lastLepResult: ");
        Serial.print(_lastLepResult);
        Serial.print(": ");
        Serial.println(textForLepResult(getLastLepResult()));
    }
}

#endif

bool LeptonFLiR::waitCommandBegin(int timeout) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("    LeptonFLiR::waitCommandBegin");
#endif

    _lastLepResult = 0;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  ");
#endif
    uint16_t status;
    if (readRegister(LEP_I2C_STATUS_REG, &status))
        return false;

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK))
        return true;

    unsigned long endTime = millis() + (unsigned long)timeout;

    while ((status & LEP_I2C_STATUS_BUSY_BIT_MASK) && (timeout <= 0 || millis() < endTime)) {
#ifdef LEPFLIR_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.print("  ");
#endif

        if (readRegister(LEP_I2C_STATUS_REG, &status))
            return false;
    }

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK))
        return true;
    else {
        _lastLepResult = LEP_TIMEOUT_ERROR;
        return false;
    }
}

bool LeptonFLiR::waitCommandFinish(int timeout) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("    LeptonFLiR::waitCommandFinish");
#endif

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  ");
#endif
    uint16_t status;
    if (readRegister(LEP_I2C_STATUS_REG, &status))
        return false;

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK)) {
        _lastLepResult = (byte)((status & LEP_I2C_STATUS_ERROR_CODE_BIT_MASK) >> LEP_I2C_STATUS_ERROR_CODE_BIT_SHIFT);
        return true;
    }

    unsigned long endTime = millis() + (unsigned long)timeout;

    while ((status & LEP_I2C_STATUS_BUSY_BIT_MASK) && (timeout <= 0 || millis() < endTime)) {
#ifdef LEPFLIR_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.print("  ");
#endif

        if (readRegister(LEP_I2C_STATUS_REG, &status))
            return false;
    }

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK)) {
        _lastLepResult = (byte)((status & LEP_I2C_STATUS_ERROR_CODE_BIT_MASK) >> LEP_I2C_STATUS_ERROR_CODE_BIT_SHIFT);
        return true;
    }
    else {
        _lastLepResult = LEP_TIMEOUT_ERROR;
        return false;
    }
}

uint16_t LeptonFLiR::cmdCode(uint16_t cmdID, uint16_t cmdType) {
    return (cmdID & LEP_I2C_COMMAND_MODULE_ID_BIT_MASK) | (cmdID & LEP_I2C_COMMAND_ID_BIT_MASK) | (cmdType & LEP_I2C_COMMAND_TYPE_BIT_MASK);
}

void LeptonFLiR::sendCommand(uint16_t cmdCode) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, NULL, 0) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint16_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, &value, 1) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint32_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, (uint16_t *)&value, 2) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint16_t *dataWords, int dataLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, dataWords, dataLength) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::receiveCommand(uint16_t cmdCode, uint16_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

            if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

                readDataRegister(value, 1);
            }
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::receiveCommand(uint16_t cmdCode, uint32_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

            if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

                readDataRegister((uint16_t *)value, 2);
            }
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::receiveCommand(uint16_t cmdCode, uint16_t *readWords, int maxLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

            if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

                readDataRegister(readWords, maxLength);
            }
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

int LeptonFLiR::writeCmdRegister(uint16_t cmdCode, uint16_t *dataWords, int dataLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("    LeptonFLiR::writeCmdRegister cmdCode: 0x");
    Serial.print(cmdCode, HEX);
    Serial.print(", dataWords[");
    Serial.print(dataLength);
    Serial.print("]: ");
    for (int i = 0; i < dataLength; ++i) {
        Serial.print(i > 0 ? "-0x" : "0x");
        Serial.print(dataWords[i], HEX);
    }
    Serial.println("");
#endif

    // In avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
    // how many words can be written at once. Therefore, we loop around until all words
    // have been written out into their registers.

    if (dataWords && dataLength) {
        i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
        i2cWire_write16(LEP_I2C_DATA_LENGTH_REG);
        i2cWire_write16(dataLength);
        if (i2cWire_endTransmission())
            return _lastI2CError;

        int maxLength = BUFFER_LENGTH / 2;
        int writeLength = min(maxLength, dataLength);
        uint16_t regAddress = dataLength <= 16 ? LEP_I2C_DATA_0_REG : LEP_I2C_DATA_BUFFER;

        while (dataLength > 0) {
            i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
            i2cWire_write16(regAddress);

            while (writeLength-- > 0)
                i2cWire_write16(*dataWords++);

            if (i2cWire_endTransmission())
                return _lastI2CError;

            regAddress += maxLength * 0x02;
            dataLength -= maxLength;
            writeLength = min(maxLength, dataLength);
        }
    }

    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    i2cWire_write16(LEP_I2C_COMMAND_REG);
    i2cWire_write16(cmdCode);
    return i2cWire_endTransmission();
}

int LeptonFLiR::readDataRegister(uint16_t *readWords, int maxLength) {
    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    i2cWire_write16(LEP_I2C_DATA_LENGTH_REG);
    if (i2cWire_endTransmission())
        return _lastI2CError;

    int bytesRead = i2cWire_requestFrom(LEP_I2C_DEVICE_ADDRESS, 2);
    if (bytesRead != 2) {
        while (bytesRead-- > 0)
            i2cWire_read();
        return (_lastI2CError = 4);
    }

    int readLength = i2cWire_read16();

    if (readLength == 0)
        return (_lastI2CError = 4);

    // In avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
    // how many words can be read at once. Therefore, we loop around until all words
    // have been read out from their registers.

    bytesRead = i2cWire_requestFrom(LEP_I2C_DEVICE_ADDRESS, min(BUFFER_LENGTH, readLength));

    while (bytesRead > 0 && readLength > 0) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        int origWordsRead = bytesRead / 2;
        int origReadLength = readLength / 2;
        int origMaxLength = maxLength;
        uint16_t *origReadWords = readWords;
#endif

        while (bytesRead > 1 && readLength > 1 && maxLength > 0) {
            *readWords++ = i2cWire_read16();
            bytesRead -= 2; readLength -= 2; --maxLength;
        }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.print("      LeptonFLiR::readDataRegister readWords[");
        if (origWordsRead == origReadLength && origReadLength == origMaxLength) {
            Serial.print(origWordsRead);
        }
        else if (origWordsRead != origReadLength && origReadLength == origMaxLength) {
            Serial.print("r:");
            Serial.print(origWordsRead);
            Serial.print(",lm:");
            Serial.print(origReadLength);
        }
        else {
            Serial.print("r:");
            Serial.print(origWordsRead);
            Serial.print(",l:");
            Serial.print(origReadLength);
            Serial.print(",m:");
            Serial.print(origMaxLength);
        }
        Serial.print("]: ");
        for (int i = 0; i < origWordsRead; ++i) {
            Serial.print(i > 0 ? "-0x" : "0x");
            Serial.print(origReadWords[i], HEX);
        }
        Serial.println("");
#endif

        if (readLength > 0)
            bytesRead += i2cWire_requestFrom(LEP_I2C_DEVICE_ADDRESS, min(BUFFER_LENGTH, readLength));
    }

    while (bytesRead-- > 0)
        i2cWire_read();

    while (maxLength-- > 0)
        *readWords++ = 0;

    return (_lastI2CError = readLength ? 4 : 0);
}

int LeptonFLiR::writeRegister(uint16_t regAddress, uint16_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("    LeptonFLiR::writeRegister regAddress: 0x");
    Serial.print(regAddress, HEX);
    Serial.print(", value: 0x");
    Serial.println(value, HEX);
#endif

    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    i2cWire_write16(regAddress);
    i2cWire_write16(value);
    return i2cWire_endTransmission();
}

int LeptonFLiR::readRegister(uint16_t regAddress, uint16_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("    LeptonFLiR::readRegister regAddress: 0x");
    Serial.println(regAddress, HEX);
#endif

    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    i2cWire_write16(regAddress);
    if (i2cWire_endTransmission())
        return _lastI2CError;

    int bytesRead = i2cWire_requestFrom(LEP_I2C_DEVICE_ADDRESS, 2);
    if (bytesRead != 2) {
        while (bytesRead-- > 0)
            i2cWire_read();
        return (_lastI2CError = 4);
    }

    *value = i2cWire_read16();

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("      LeptonFLiR::readRegister retVal: 0x");
    Serial.println(*value, HEX);
#endif

    return _lastI2CError;
}

#ifdef LEPFLIR_USE_SOFTWARE_I2C
bool __attribute__((noinline)) i2c_start(uint8_t addr);
void __attribute__((noinline)) i2c_stop(void) asm("ass_i2c_stop");
bool __attribute__((noinline)) i2c_write(uint8_t value) asm("ass_i2c_write");
uint8_t __attribute__((noinline)) i2c_read(bool last);
#endif

void LeptonFLiR::i2cWire_beginTransmission(uint8_t addr) {
    _lastI2CError = 0;
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    _i2cWire->beginTransmission(addr);
#else
    i2c_start(addr);
#endif
}

uint8_t LeptonFLiR::i2cWire_endTransmission(void) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return (_lastI2CError = _i2cWire->endTransmission());
#else
    i2c_stop();
    return (_lastI2CError = 0);
#endif
}

uint8_t LeptonFLiR::i2cWire_requestFrom(uint8_t addr, uint8_t len) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cWire->requestFrom(addr, len);
#else
    i2c_start(addr | 0x01);
    return (_readBytes = len);
#endif
}

size_t LeptonFLiR::i2cWire_write(uint8_t data) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cWire->write(data);
#else
    return (size_t)i2c_write(data);
#endif
}

size_t LeptonFLiR::i2cWire_write16(uint16_t data) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cWire->write(highByte(data)) + _i2cWire->write(lowByte(data));
#else
    return (size_t)i2c_write(highByte(data)) + (size_t)i2c_write(lowByte(data));
#endif
}

uint8_t LeptonFLiR::i2cWire_read(void) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return (uint8_t)(_i2cWire->read() & 0xFF);
#else
    if (_readBytes > 1) {
        _readByes -= 1;
        return (uint8_t)(i2c_read(false) & 0xFF);
    }
    else {
        _readBytes = 0;
        return (uint8_t)(i2c_read(true) & 0xFF);
    }
#endif
}

uint16_t LeptonFLiR::i2cWire_read16(void) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return ((uint16_t)(_i2cWire->read() & 0xFF) << 8) | (uint16_t)(_i2cWire->read() & 0xFF);
#else
    if (_readBytes > 2) {
        readBytes -= 2;
        return ((uint16_t)(i2c_read(false) & 0xFF) << 8) | (uint16_t)(i2c_read(false) & 0xFF);
    }
    else {
        _readBytes = 0;
        return ((uint16_t)(i2c_read(false) & 0xFF) << 8) | (uint16_t)(i2c_read(true) & 0xFF);
    }
#endif
}

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

void LeptonFLiR::printModuleInfo() {
    char buffer[80];

    Serial.println(""); Serial.println(" ~~~ LeptonFLiR Module Info ~~~");

    Serial.println(""); Serial.println("Chip Select Pin:");
    Serial.print("D");
    Serial.print(_spiCSPin);
    Serial.println(" (active-low)");

    Serial.println(""); Serial.println("SPI Port Speed:");
    for (int divisor = 2; divisor <= 128; divisor *= 2) {
        if (F_CPU / (float)divisor <= LEPFLIR_SPI_MAX_SPEED + 0.00001f || divisor == 128) {
            Serial.print(roundf((F_CPU / (float)divisor) / 1000.0f) / 1000.0f);
            Serial.print("MHz (SPI_CLOCK_DIV");
            Serial.print(divisor);
            Serial.print(")");
            break;
        }
    }
    if (F_CPU / 2.0f < LEPFLIR_SPI_MIN_SPEED - 0.00001f)
        Serial.println(" <speed too low>");
    else if (F_CPU / 128.0f > LEPFLIR_SPI_MAX_SPEED + 0.00001f)
        Serial.println(" <speed too high>");
    else
        Serial.println("");

    Serial.println(""); Serial.println("Image Storage Mode:");
    Serial.print(_storageMode);
    Serial.print(": ");
    switch (_storageMode) {
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
            Serial.println("LeptonFLiR_ImageStorageMode_80x60_16bpp"); break;
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
            Serial.println("LeptonFLiR_ImageStorageMode_80x60_8bpp"); break;
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
            Serial.println("LeptonFLiR_ImageStorageMode_40x30_16bpp"); break;
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
            Serial.println("LeptonFLiR_ImageStorageMode_40x30_8bpp"); break;
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
            Serial.println("LeptonFLiR_ImageStorageMode_20x15_16bpp"); break;
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            Serial.println("LeptonFLiR_ImageStorageMode_20x15_8bpp"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Temperature Mode:");
    Serial.print(_tempMode);
    Serial.print(": ");
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            Serial.println("LeptonFLiR_TemperatureMode_Celsius"); break;
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            Serial.println("LeptonFLiR_TemperatureMode_Fahrenheit"); break;
        case LeptonFLiR_TemperatureMode_Kelvin:
            Serial.println("LeptonFLiR_TemperatureMode_Kelvin"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Memory Footprint:");
    int mallocOffset = 0;
#ifndef LEPFLIR_DISABLE_ALIGNED_MALLOC
    mallocOffset = 15;
#endif
    Serial.print("Image Data: ");
    Serial.print(_imageData ? getImageTotalBytes() + mallocOffset : 0);
    Serial.print("B, SPI Frame Data: ");
    Serial.print(_spiFrameData ? getSPIFrameTotalBytes() + mallocOffset : 0);
    Serial.print("B, Telemetry Data: ");
    Serial.print(_telemetryData ? LEPFLIR_SPI_FRAME_PACKET_SIZE : 0);
    Serial.print("B, Total: ");
    Serial.print((_imageData ? getImageTotalBytes() + mallocOffset : 0) + (_spiFrameData ? getSPIFrameTotalBytes() + mallocOffset : 0) + (_telemetryData ? LEPFLIR_SPI_FRAME_PACKET_SIZE : 0));
    Serial.println("B");

    Serial.println(""); Serial.println("Power Register:");
    uint16_t powerReg; readRegister(LEP_I2C_POWER_REG, &powerReg);
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
    Serial.print("0x");
    Serial.println(powerReg, HEX);

    Serial.println(""); Serial.println("Status Register:");
    uint16_t statusReg; readRegister(LEP_I2C_STATUS_REG, &statusReg);
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
    Serial.print("0x");
    Serial.println(statusReg, HEX);

    Serial.println(""); Serial.println("AGC Enabled:");
    Serial.println(agc_getAGCEnabled() ? "enabled" : "disabled");

    Serial.println(""); Serial.println("AGC Policy:");
    LEP_AGC_POLICY policy = agc_getAGCPolicy();
    Serial.print(policy);
    Serial.print(": ");
    switch (policy) {
        case LEP_AGC_LINEAR:
            Serial.println("LEP_AGC_LINEAR"); break;
        case LEP_AGC_HEQ:
            Serial.println("LEP_AGC_HEQ"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("AGC HEQ Scale Factor:");
    LEP_AGC_HEQ_SCALE_FACTOR factor = agc_getHEQScaleFactor();
    Serial.print(factor);
    Serial.print(": ");
    switch (factor) {
        case LEP_AGC_SCALE_TO_8_BITS:
            Serial.println("LEP_AGC_SCALE_TO_8_BITS"); break;
        case LEP_AGC_SCALE_TO_14_BITS:
            Serial.println("LEP_AGC_SCALE_TO_14_BITS"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("AGC Calculation Enabled:");
    Serial.println(agc_getAGCCalcEnabled() ? "enabled" : "disabled");

    Serial.println(""); Serial.println("SYS Camera Status:");
    LEP_SYS_CAM_STATUS_STATES camStatus = sys_getCameraStatus();
    Serial.print(camStatus);
    Serial.print(": ");
    switch (camStatus) {
        case LEP_SYSTEM_READY:
            Serial.println("LEP_SYSTEM_READY"); break;
        case LEP_SYSTEM_INITIALIZING:
            Serial.println("LEP_SYSTEM_INITIALIZING"); break;
        case LEP_SYSTEM_IN_LOW_POWER_MODE:
            Serial.println("LEP_SYSTEM_IN_LOW_POWER_MODE"); break;
        case LEP_SYSTEM_GOING_INTO_STANDBY:
            Serial.println("LEP_SYSTEM_GOING_INTO_STANDBY"); break;
        case LEP_SYSTEM_FLAT_FIELD_IN_PROCESS:
            Serial.println("LEP_SYSTEM_FLAT_FIELD_IN_PROCESS"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("FLiR Serial Number:");
    sys_getFlirSerialNumber(buffer, 80);
    Serial.println(buffer);

    Serial.println(""); Serial.println("Customer Serial Number:");
    sys_getCustomerSerialNumber(buffer, 80);
    Serial.println(buffer);

    Serial.println(""); Serial.println("Camera Uptime:");
    Serial.print(sys_getCameraUptime());
    Serial.println(" ms");

    Serial.println(""); Serial.println("Sys Aux Temperature:");
    Serial.print(sys_getAuxTemperature());
    Serial.println(getTemperatureSymbol());

    Serial.println(""); Serial.println("Sys FPA Temperature:");
    Serial.print(sys_getFPATemperature());
    Serial.println(getTemperatureSymbol());

    Serial.println(""); Serial.println("Telemetry Enabled:");
    Serial.println(sys_getTelemetryEnabled() ? "enabled" : "disabled");

    Serial.println(""); Serial.println("Vid Polarity:");
    LEP_VID_POLARITY polarity = vid_getPolarity();
    Serial.print(polarity);
    Serial.print(": ");
    switch (polarity) {
        case LEP_VID_WHITE_HOT:
            Serial.println("LEP_VID_WHITE_HOT"); break;
        case LEP_VID_BLACK_HOT:
            Serial.println("LEP_VID_BLACK_HOT"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Vid Pseudo Color Lookup Table:");
    LEP_VID_PCOLOR_LUT table = vid_getPseudoColorLUT();
    Serial.print(table);
    Serial.print(": ");
    switch (table) {
        case LEP_VID_WHEEL6_LUT:
            Serial.println("LEP_VID_WHEEL6_LUT"); break;
        case LEP_VID_FUSION_LUT:
            Serial.println("LEP_VID_FUSION_LUT"); break;
        case LEP_VID_RAINBOW_LUT:
            Serial.println("LEP_VID_RAINBOW_LUT"); break;
        case LEP_VID_GLOBOW_LUT:
            Serial.println("LEP_VID_GLOBOW_LUT"); break;
        case LEP_VID_COLOR_LUT:
            Serial.println("LEP_VID_COLOR_LUT"); break;
        case LEP_VID_ICE_FIRE_LUT:
            Serial.println("LEP_VID_ICE_FIRE_LUT"); break;
        case LEP_VID_RAIN_LUT:
            Serial.println("LEP_VID_RAIN_LUT"); break;
        case LEP_VID_USER_LUT:
            Serial.println("LEP_VID_USER_LUT"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Vid Focus Calculation Enabled:");
    Serial.println(vid_getFocusCalcEnabled() ? "enabled" : "disabled");

    Serial.println(""); Serial.println("Vid Freeze Enabled:");
    Serial.println(vid_getFreezeEnabled() ? "enabled" : "disabled");
}

#endif
