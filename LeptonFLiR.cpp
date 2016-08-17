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

    Lepton-FLiR-Arduino - Version 0.5
*/

#include "LeptonFLiR.h"
#if (defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)) && !defined(LEPFLIR_DISABLE_SCHEDULER)
#include "Scheduler.h"
#define LEPFLIR_USE_SCHEDULER           1
#endif

#define LEPFLIR_GEN_CMD_TIMEOUT         5000 // Timeout for commands to be processed

#ifndef LEPFLIR_DISABLE_ALIGNED_MALLOC
static inline int roundUpVal16(int val) { return ((val + 15) & -16); }
static inline uint8_t *roundUpPtr16(uint8_t *ptr) { return (uint8_t *)(((uintptr_t)ptr + 15) & 0xF); }
static inline uint8_t *roundUpMalloc16(int size) { return (uint8_t *)malloc((size_t)(size + 15)); }
static inline uint8_t *roundUpSpiFrame16(uint8_t *spiFrame) { return roundUpPtr16(spiFrame) + 16 - 4; }
#else
static inline int roundUpVal16(int val) { return val; }
static inline uint8_t *roundUpPtr16(uint8_t *ptr) { return ptr; }
static inline uint8_t *roundUpMalloc16(int size) { return (uint8_t *)malloc((size_t)size); }
static inline uint8_t *roundUpSpiFrame16(uint8_t *spiFrame) { return spiFrame; }
#endif

#ifndef LEPFLIR_USE_SOFTWARE_I2C
LeptonFLiR::LeptonFLiR(TwoWire& i2cWire, uint8_t spiCSPin) {
    _i2cWire = &i2cWire;
#else
LeptonFLiR::LeptonFLiR(uint8_t spiCSPin) {
#endif
    _spiCSPin = spiCSPin;
    _spiSettings = SPISettings(20000000, MSBFIRST, SPI_MODE3);
    _storageMode = LeptonFLiR_ImageStorageMode_Count;
    _imageData = _spiFrameData = _telemetryData = NULL;
    _isReadingNextFrame = false;
    _lastI2CError = _lastErrorCode = 0;
}

LeptonFLiR::~LeptonFLiR() {
    if (_imageData) free(_imageData);
    if (_spiFrameData) free(_spiFrameData);
    if (_telemetryData) free(_telemetryData);
}

void LeptonFLiR::init(LeptonFLiR_ImageStorageMode storageMode) {
    _storageMode = (LeptonFLiR_ImageStorageMode)constrain((int)storageMode, 0, (int)LeptonFLiR_ImageStorageMode_Count - 1);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("LeptonFLiR::init spiCSPin: ");
    Serial.print(_spiCSPin);
    Serial.print(", storageMode: ");
    Serial.println(storageMode);
#endif

    pinMode(_spiCSPin, OUTPUT);
    digitalWrite(_spiCSPin, HIGH);

    _imageData = roundUpMalloc16(((getImageHeight() - 1) * getImagePitch()) + (getImageWidth() * getImageBpp()));
    _spiFrameData = roundUpMalloc16(getImageDivFactor() * roundUpVal16(LEP_SPI_FRAME_SIZE));

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::init ImageData: 0x");
    Serial.print((uintptr_t)_imageData, HEX);
    Serial.print(", SPIFrameData: 0x");
    Serial.println((uintptr_t)_spiFrameData, HEX);
    Serial.print("  LeptonFLiR::init SPIPortSpeed: ");
    for (int divisor = 2; divisor <= 128; divisor *= 2) {
        if ((double)F_CPU / divisor <= 20000000.0 + 0.000001) {
            Serial.print(round(((double)F_CPU / divisor) / 1000.0) / 1000.0);
            Serial.print("MHz (SPI_CLOCK_DIV");
            Serial.print(divisor);
            Serial.println(")");
            break;
        }
    }
#endif
}

LeptonFLiR_ImageStorageMode LeptonFLiR::getImageStorageMode() {
    return _storageMode;
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

int LeptonFLiR::getImageDivFactor() {
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

uint8_t *LeptonFLiR::getImageData() {
    return !_isReadingNextFrame ? roundUpPtr16(_imageData) : NULL;
}

uint8_t *LeptonFLiR::getImageDataRow(int row) {
    return !_isReadingNextFrame ? (roundUpPtr16(_imageData) + (getImagePitch() * row)) : NULL;
}

uint8_t *LeptonFLiR::getImageDataRowCol(int row, int col) {
    return !_isReadingNextFrame ? (roundUpPtr16(_imageData) + (getImagePitch() * row) + (getImageBpp() * col)) : NULL;
}

uint8_t *LeptonFLiR::getSPIFrameDataRow(int row) {
    return roundUpSpiFrame16(_spiFrameData) + (roundUpVal16(LEP_SPI_FRAME_SIZE) * row);
}

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

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

static void printSPIFrame(uint8_t *spiFrame, uint8_t *pxlData) {
    if (spiFrame) {
        Serial.print("ID: 0x");
        Serial.print(((uint16_t *)spiFrame)[0], HEX);
        Serial.print(" CRC: 0x");
        Serial.print(((uint16_t *)spiFrame)[1], HEX);
        if (!pxlData) Serial.println("");
    }
    
    if (pxlData) {
        if (spiFrame) Serial.print(" ");
        Serial.print("Data: ");
        for (int i = 0; i < 5; ++i) {
            Serial.print(i > 0 ? "-0x" : "0x");
            Serial.print(((uint16_t *)pxlData)[i], HEX);
        }
        Serial.print("...");
        for (int i = 75; i < 80; ++i) {
            Serial.print(i > 75 ? "-0x" : "0x");
            Serial.print(((uint16_t *)pxlData)[i], HEX);
        }
        Serial.println("");
    }
}

#endif

bool LeptonFLiR::readNextFrame() {
    if (!_isReadingNextFrame) {
        _isReadingNextFrame = true;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.println("LeptonFLiR::readNextFrame");
#endif

        bool agcEnabled, telemetryEnabled, cameraBooted; LEP_SYS_TELEMETRY_LOCATION telemetryLocation;
        {   uint16_t value = 0;
            receiveCommand(commandCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &value);
            agcEnabled = value > 0;

            value = 0; receiveCommand(commandCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &value);
            telemetryEnabled = value > 0;
            
            value = 0; receiveCommand(commandCode(LEP_CID_SYS_TELEMETRY_LOCATION, LEP_I2C_COMMAND_TYPE_GET), &value);
            telemetryLocation = (value != (uint32_t)LEP_TELEMETRY_LOCATION_FOOTER ? LEP_TELEMETRY_LOCATION_HEADER : LEP_TELEMETRY_LOCATION_FOOTER);

            value = 0; readRegister(LEP_I2C_STATUS_REG, &value, 1, 1);
            cameraBooted = (value & LEP_I2C_STATUS_BOOT_MODE_BIT_MASK) > 0 && (value & LEP_I2C_STATUS_BOOT_STATUS_BIT_MASK) > 0;
        }

        if (!cameraBooted) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
            Serial.println("  LeptonFLiR::readNextFrame Camera has not yet booted. Aborting.");
#endif
            _isReadingNextFrame = false;
            return false;
        }

        if (telemetryEnabled && !_telemetryData) {
            _telemetryData = (uint8_t *)malloc(LEP_SPI_FRAME_SIZE);
            _telemetryData[0] = 0x0F; // initialize as discard packet
        }
        else if (!telemetryEnabled && _telemetryData) {
            free(_telemetryData);
            _telemetryData = NULL;
        }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.print("  LeptonFLiR::readNextFrame AGC: ");
        Serial.print(agcEnabled ? "enabled" : "disabled");
        Serial.print(", Telemetry: ");
        if (telemetryEnabled) {
            Serial.print("enabled, Location: ");
            Serial.println(telemetryLocation == LEP_TELEMETRY_LOCATION_HEADER ? "header" : "footer");
        }
        else
            Serial.println("disabled");
#endif

        uint_fast8_t readLines = 0;
        uint_fast8_t imgRows = getImageHeight();
        uint_fast8_t currImgRow = 0;
        uint_fast8_t spiRows = getImageDivFactor();
        uint_fast8_t currSpiRow = 0;
        uint_fast8_t teleRows = (telemetryEnabled * 3);
        uint_fast8_t currTeleRow = 0;
        uint_fast8_t framesSkipped = 0;
        uint_fast16_t packetsRead = -1;
        bool packetHeaderRead = false;
        //bool wroteTeleData = false;

        SPI.beginTransaction(_spiSettings);
        digitalWrite(_spiCSPin, LOW);

        while (currImgRow < imgRows || currTeleRow < teleRows) {
            uint8_t *spiFrame = getSPIFrameDataRow(currSpiRow);
            
            if (!packetHeaderRead) {
                ++packetsRead;
                SPI.transfer(spiFrame, 4);
                spiFrame[0] &= 0x0F;
            }
            else
                packetHeaderRead = false;

            if (spiFrame[0] == 0x00 && spiFrame[1] == readLines) { // Image packet
                uint8_t *pxlData = (_storageMode == LeptonFLiR_ImageStorageMode_80x60_16bpp ? roundUpPtr16(_imageData) + (getImagePitch() * readLines) : &spiFrame[4]);

                SPI.transfer(pxlData, LEP_SPI_FRAME_SIZE - 4);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                Serial.println("    LeptonFLiR::readNextFrame VoSPI Image Packet:");
                Serial.print("      ");  printSPIFrame(spiFrame, pxlData);
#endif

                ++readLines; ++currSpiRow;

                if (currSpiRow == spiRows) {
                    if (_storageMode != LeptonFLiR_ImageStorageMode_80x60_16bpp) {
                        spiFrame = getSPIFrameDataRow(0) + 4;
                        pxlData = roundUpPtr16(_imageData) + (getImagePitch() * currImgRow);

                        uint_fast8_t imgWidth = getImageWidth();
                        uint_fast8_t imgBpp = getImageBpp();

                        uint_fast32_t divisor = (spiRows * spiRows) * (!agcEnabled && imgBpp == 1 ? 64 : 1);
                        uint_fast32_t clamp = (!agcEnabled && imgBpp == 2 ? 0x3FFF : 0x00FF);

                        while (imgWidth-- > 0) {
                            uint_fast32_t total = 0;

                            uint_fast8_t y = spiRows;
                            uint8_t *spiYFrame = spiFrame;
                            while (y-- > 0) {

                                uint_fast8_t x = spiRows;
                                uint16_t *spiXFrame = (uint16_t *)spiYFrame;
                                while (x-- > 0)
                                    total += *spiXFrame++;

                                spiYFrame += roundUpVal16(LEP_SPI_FRAME_SIZE);
                            }

                            if (imgBpp == 2)
                                *((uint16_t *)pxlData) = (uint16_t)constrain(total / divisor, 0, clamp);
                            else
                                *((uint8_t *)pxlData) = (uint8_t)constrain(total / divisor, 0, clamp);
                            pxlData += imgBpp;
                            spiFrame += 2 * spiRows;
                        }
                    }

                    ++currImgRow; currSpiRow = 0;
                }
            }
            else if (spiFrame[0] != 0x0F && teleRows && currTeleRow < 3 &&
                ((telemetryLocation == LEP_TELEMETRY_LOCATION_HEADER && readLines == 0) ||
                (telemetryLocation == LEP_TELEMETRY_LOCATION_FOOTER && readLines == 60))) { // Telemetry packet
                if (currTeleRow == 0) {
                    memcpy(_telemetryData, spiFrame, 4);
                    SPI.transfer(&_telemetryData[4], LEP_SPI_FRAME_SIZE - 4);
                    //wroteTeleData = true;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                    Serial.println("    LeptonFLiR::readNextFrame VoSPI Telemetry(A) Packet:");
                    Serial.print("      ");  printSPIFrame(_telemetryData, &_telemetryData[4]);
#endif
                }
                else {
                    SPI.transfer(&spiFrame[4], LEP_SPI_FRAME_SIZE - 4);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                    Serial.print("    LeptonFLiR::readNextFrame VoSPI Telemetry(");
                    Serial.print(currTeleRow == 1 ? "B" : "C");
                    Serial.println(") Packet:");
                    Serial.print("      ");  printSPIFrame(spiFrame, NULL);
#endif
                }

                ++currTeleRow;
            }
            else { // Discard packet
                SPI.transfer(&spiFrame[4], LEP_SPI_FRAME_SIZE - 4);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                Serial.println("    LeptonFLiR::readNextFrame VoSPI Discard Packet:");
                Serial.print("      ");  printSPIFrame(spiFrame, &spiFrame[4]);
#endif

                if (packetsRead > 0) {
                    if (++framesSkipped >= 5) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                        Serial.println("  LeptonFLiR::readNextFrame Maximum frame skip reached. Aborting.");
#endif
                        digitalWrite(_spiCSPin, HIGH);
                        SPI.endTransaction();
                        _isReadingNextFrame = false;
                        return false;
                    }

                    if (spiFrame[0] == 0x0F) {
                        digitalWrite(_spiCSPin, HIGH);
                        delayTimeout(185);
                        digitalWrite(_spiCSPin, LOW);
                    }
                }

                readLines = currImgRow = currSpiRow = currTeleRow = 0;
                //if (wroteTeleData) { wroteTeleData = false; _telemetryData[0] = 0x0F; } // mark as invalid

                spiFrame = getSPIFrameDataRow(currSpiRow);
                packetHeaderRead = true;
                uint_fast8_t triesLeft = 120;
                
                while (triesLeft > 0) {
                    ++packetsRead;
                    SPI.transfer(spiFrame, 4);
                    spiFrame[0] &= 0x0F;

                    if (spiFrame[0] != 0x0F &&
                        ((spiFrame[0] == 0x00 && spiFrame[1] == 0) ||
                        (spiFrame[0] > 0x00 && teleRows && telemetryLocation == LEP_TELEMETRY_LOCATION_HEADER)))
                        break;

                    --triesLeft;
                    SPI.transfer(&spiFrame[4], LEP_SPI_FRAME_SIZE - 4);
                }

                if (triesLeft == 0) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                    Serial.println("  LeptonFLiR::readNextFrame Maximum resync retries reached. Aborting.");
#endif
                    digitalWrite(_spiCSPin, HIGH);
                    SPI.endTransaction();
                    _isReadingNextFrame = false;
                    return false;
                }
            }
        }
        
        digitalWrite(_spiCSPin, HIGH);
        SPI.endTransaction();

        _isReadingNextFrame = false;
    }

    return true;
}

void LeptonFLiR::setAGCEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("LeptonFLiR::setAGCEnabled enabled: ");
    Serial.println(enabled);
#endif

    sendCommand(commandCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::getAGCEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getAGCEnabled");
#endif

    uint16_t enabled = 0;
    receiveCommand(commandCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled > 0;
}

void LeptonFLiR::setTelemetryEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("LeptonFLiR::setTelemetryEnabled enabled: ");
    Serial.println(enabled);
#endif

    sendCommand(commandCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);

    if (enabled) {
        if (!_telemetryData) {
            _telemetryData = (uint8_t *)malloc(LEP_SPI_FRAME_SIZE);
            _telemetryData[0] = 0x0F; // initialize as discard packet
        }
    }
    else {
        if (_telemetryData) {
            free(_telemetryData);
            _telemetryData = NULL;
        }
    }
}

bool LeptonFLiR::getTelemetryEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getTelemetryEnabled");
#endif

    uint16_t enabled = 0;
    receiveCommand(commandCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled > 0;
}

uint8_t *LeptonFLiR::getTelemetryData() {
    // Don't let user have access to telemetry data if it hasn't been filled yet (that is, ID is discard packet)
    return !_isReadingNextFrame && _telemetryData && _telemetryData[0] != 0x0F ? _telemetryData : NULL;
}

uint8_t LeptonFLiR::getLastI2CError() {
    return _lastI2CError;
}

int16_t LeptonFLiR::getLastErrorCode() {
    return _lastErrorCode;
}

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

void LeptonFLiR::printModuleInfo() {
    uint16_t data[16];

    Serial.println("SYS Camera Status:");
    receiveCommand(commandCode(LEP_CID_SYS_CAM_STATUS, LEP_I2C_COMMAND_TYPE_GET), data, 16);

    Serial.println("SYS Customer Serial Number:");
    receiveCommand(commandCode(LEP_CID_SYS_CUST_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), data, 16);

    Serial.println("SYS FLiR Serial Number:");
    receiveCommand(commandCode(LEP_CID_SYS_FLIR_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), data, 16);

    Serial.println("SYS Camera Uptime:");
    receiveCommand(commandCode(LEP_CID_SYS_CAM_UPTIME, LEP_I2C_COMMAND_TYPE_GET), data, 16);
    
    Serial.println("SYS Aux Temperature Kelvin:");
    receiveCommand(commandCode(LEP_CID_SYS_AUX_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), data, 16);

    Serial.println("SYS FPA Temperature Kelvin:");
    receiveCommand(commandCode(LEP_CID_SYS_FPA_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), data, 16);

    Serial.println("AGC Enable State:");
    receiveCommand(commandCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), data, 16);

    Serial.println("SYS Telemetry Enable State:");
    receiveCommand(commandCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), data, 16);
}

#endif

bool LeptonFLiR::isBusy() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("    LeptonFLiR::isBusy");
#endif

    uint16_t status = 0;
    readRegister(LEP_I2C_STATUS_REG, &status, 1, 1);
    return status & LEP_I2C_STATUS_BUSY_BIT_MASK;
}

bool LeptonFLiR::waitBusy(int timeout) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("    LeptonFLiR::waitBusy timeout: ");
    Serial.println(timeout);
#endif
    
    if (!isBusy()) return false;

    unsigned long endTime = millis() + (unsigned long)timeout;

    while (isBusy() && (timeout <= 0 || millis() < endTime)) {
#ifdef LEPFLIR_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif
    }

    return isBusy();
}

void LeptonFLiR::receiveStatus(bool *busy, bool *bootMode, bool *bootStatus) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("    LeptonFLiR::receiveStatus");
#endif

    uint16_t status = 0;

    if (readRegister(LEP_I2C_STATUS_REG, &status, 1, 1) == 0) {
        if (busy) *busy = (status & LEP_I2C_STATUS_BUSY_BIT_MASK) > 0;
        if (bootMode) *bootMode = (status & LEP_I2C_STATUS_BOOT_MODE_BIT_MASK) > 0;
        if (bootStatus) *bootStatus = (status & LEP_I2C_STATUS_BOOT_STATUS_BIT_MASK) > 0;

        _lastErrorCode = (int8_t)((uint8_t)(((status & LEP_I2C_STATUS_ERROR_CODE_BIT_MASK) >> LEP_I2C_STATUS_ERROR_CODE_BIT_SHIFT) & 0xFF));
    }
}

uint16_t LeptonFLiR::commandCode(uint16_t cmdID, uint16_t cmdType) {
    return (cmdID & LEP_I2C_COMMAND_MODULE_ID_BIT_MASK) | (cmdID & LEP_I2C_COMMAND_ID_BIT_MASK) | (cmdType & LEP_I2C_COMMAND_TYPE_BIT_MASK);
}

void LeptonFLiR::sendCommand(uint16_t cmdCode) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitBusy(LEPFLIR_GEN_CMD_TIMEOUT)) {
        _lastErrorCode = LEP_TIMEOUT_ERROR; return;
    }

    uint16_t cmdBuffer[2] = { cmdCode, 0 };
    writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2);
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint16_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitBusy(LEPFLIR_GEN_CMD_TIMEOUT)) {
        _lastErrorCode = LEP_TIMEOUT_ERROR; return;
    }

    uint16_t cmdBuffer[3] = { cmdCode, (uint16_t)1, value };
    writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 3);
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint32_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitBusy(LEPFLIR_GEN_CMD_TIMEOUT)) {
        _lastErrorCode = LEP_TIMEOUT_ERROR; return;
    }

    uint16_t cmdBuffer[4] = { cmdCode, (uint16_t)2, (uint16_t)((value >> 16) & 0xFFFF), (uint16_t)(value & 0xFFFF) };
    writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 4);
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint16_t *dataBuffer, int dataLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitBusy(LEPFLIR_GEN_CMD_TIMEOUT)) {
        _lastErrorCode = LEP_TIMEOUT_ERROR; return;
    }

    uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)dataLength };
    writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2, dataBuffer, dataLength);
}

int LeptonFLiR::receiveCommand(uint16_t cmdCode, uint16_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitBusy(LEPFLIR_GEN_CMD_TIMEOUT)) {
        _lastErrorCode = LEP_TIMEOUT_ERROR; return 0;
    }

    uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)0 };
    if (writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2) == 0) {

        uint16_t respLength;
        if (readRegister(LEP_I2C_DATA_LENGTH_REG, &respLength, 1, 1) == 0) {
            respLength = constrain(respLength, 0, 16);

            if ((respLength == 1 && readRegister((uint16_t *)value, 1, 1) == 0) ||
                (respLength > 1 && readRegister(LEP_I2C_DATA_0_REG + ((respLength - 1) * 0x02), (uint16_t *)value, 1, 1) == 0))
                return 1;
            else
                _lastErrorCode = LEP_COMM_COUNT_ERROR;
        }
    }

    return 0;
}

int LeptonFLiR::receiveCommand(uint16_t cmdCode, uint32_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitBusy(LEPFLIR_GEN_CMD_TIMEOUT)) {
        _lastErrorCode = LEP_TIMEOUT_ERROR; return 0;
    }

    uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)0 };
    if (writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2) == 0) {

        uint16_t respLength;
        if (readRegister(LEP_I2C_DATA_LENGTH_REG, &respLength, 1, 1) == 0) {
            respLength = constrain(respLength, 0, 16);

            if ((respLength == 2 && readRegister((uint16_t *)value, 2, 2) == 0) ||
                (respLength > 2 && readRegister(LEP_I2C_DATA_0_REG + ((respLength - 2) * 0x02), (uint16_t *)value, 2, 2) == 0))
                return 2;
            else
                _lastErrorCode = LEP_COMM_COUNT_ERROR;
        }
    }

    return 0;
}

int LeptonFLiR::receiveCommand(uint16_t cmdCode, uint16_t *respBuffer, int maxLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitBusy(LEPFLIR_GEN_CMD_TIMEOUT)) {
        _lastErrorCode = LEP_TIMEOUT_ERROR; return 0;
    }

    uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)0 };
    if (writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2) == 0) {

        uint16_t respLength;
        if (readRegister(LEP_I2C_DATA_LENGTH_REG, &respLength, 1, 1) == 0) {
            respLength = constrain(respLength, 0, 16);

            if (respLength > 0 && readRegister(respBuffer, respLength, maxLength) == 0)
                return respLength;
            else
                _lastErrorCode = LEP_COMM_COUNT_ERROR;
        }
    }

    return 0;
}

int LeptonFLiR::sendReceiveCommand(uint16_t cmdCode, uint16_t *dataBuffer, int dataLength, uint16_t *respBuffer, int maxLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendReceiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitBusy(LEPFLIR_GEN_CMD_TIMEOUT)) {
        _lastErrorCode = LEP_TIMEOUT_ERROR; return 0;
    }

    uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)dataLength };
    if (writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2, dataBuffer, dataLength) == 0) {

        uint16_t respLength;
        if (readRegister(LEP_I2C_DATA_LENGTH_REG, &respLength, 1, 1) == 0) {
            respLength = constrain(respLength, 0, 16);

            if (respLength > 0 && readRegister(respBuffer, respLength, maxLength) == 0)
                return respLength;
            else
                _lastErrorCode = LEP_COMM_COUNT_ERROR;
        }
    }

    return 0;
}

int LeptonFLiR::writeRegister(uint16_t regAddress, uint16_t *dataBuffer, int dataLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("    LeptonFLiR::writeRegister regAddress: 0x");
    Serial.print(regAddress, HEX);
    Serial.print(", DataBuffer[");
    Serial.print(dataLength);
    Serial.print("]: ");
    for (int i = 0; i < dataLength; ++i) {
        Serial.print(i > 0 ? "-0x" : "0x");
        Serial.print(dataBuffer[i], HEX);
    }
    Serial.println("");
#endif

    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);

    i2cWire_write(highByte(regAddress));
    i2cWire_write(lowByte(regAddress));

    while (dataLength-- > 0) {
        i2cWire_write(highByte(*dataBuffer));
        i2cWire_write(lowByte(*dataBuffer++));
    }

    uint8_t retStat = i2cWire_endTransmission();

    if (regAddress == LEP_I2C_COMMAND_REG)
        receiveStatus();
    return retStat || _lastErrorCode;
}

int LeptonFLiR::writeRegister(uint16_t regAddress, uint16_t *dataBuffer1, int dataLength1, uint16_t *dataBuffer2, int dataLength2) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("    LeptonFLiR::writeRegister regAddress: 0x");
    Serial.print(regAddress, HEX);
    Serial.print(", DataBuffer[");
    Serial.print(dataLength1 + dataLength2);
    Serial.print("]: ");
    for (int i = 0; i < dataLength1 + dataLength2; ++i) {
        Serial.print(i > 0 ? "-0x" : "0x");
        Serial.print(i < dataLength1 ? dataBuffer1[i] : dataBuffer2[i - dataLength1], HEX);
    }
    Serial.println("");
#endif

    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);

    i2cWire_write(highByte(regAddress));
    i2cWire_write(lowByte(regAddress));

    while (dataLength1-- > 0) {
        i2cWire_write(highByte(*dataBuffer1));
        i2cWire_write(lowByte(*dataBuffer1++));
    }

    while (dataLength2-- > 0) {
        i2cWire_write(highByte(*dataBuffer2));
        i2cWire_write(lowByte(*dataBuffer2++));
    }

    uint8_t retStat = i2cWire_endTransmission();

    if (regAddress == LEP_I2C_COMMAND_REG)
        receiveStatus();
    return retStat || _lastErrorCode;
}

int LeptonFLiR::readRegister(uint16_t regAddress, uint16_t *respBuffer, int respLength, int maxLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    if (regAddress == LEP_I2C_STATUS_REG) Serial.print("  ");
    Serial.print("    LeptonFLiR::readRegister regAddress: 0x");
    Serial.println(regAddress, HEX);
#endif

    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);

    i2cWire_write(highByte(regAddress));
    i2cWire_write(lowByte(regAddress));

    i2cWire_endTransmission();

    if (_lastI2CError) return _lastI2CError;
    return readRegister(respBuffer, respLength, maxLength);
}

int LeptonFLiR::readRegister(uint16_t *respBuffer, int respLength, int maxLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("      LeptonFLiR::readRegister respLength: ");
    Serial.print(respLength);
    Serial.print(", maxLength: ");
    Serial.println(maxLength);
#endif

    int wordsRead = i2cWire_requestFrom(LEP_I2C_DEVICE_ADDRESS, respLength << 1) >> 1;

    if (wordsRead > 0) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        int origWordsRead = wordsRead;
        uint16_t *origRespBuffer = respBuffer;
#endif

        while (wordsRead-- > 0 && maxLength-- > 0) {
            *respBuffer = ((uint16_t)i2cWire_read() << 8) & 0xFF00;
            *respBuffer++ |= (uint16_t)i2cWire_read() & 0x00FF;
        }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.print("      LeptonFLiR::readRegister respBuffer[");
        Serial.print(origWordsRead);
        Serial.print("]: ");
        for (int i = 0; i < origWordsRead; ++i) {
            Serial.print(i > 0 ? "-0x" : "0x");
            Serial.print(origRespBuffer[i], HEX);
        }
        Serial.println("");
#endif

        return _lastI2CError ? : 0;
    }

    return (_lastErrorCode = LEP_COMM_COUNT_ERROR);
}

#ifdef LEPFLIR_USE_SOFTWARE_I2C
bool __attribute__((noinline)) i2c_start(uint8_t addr);
void __attribute__((noinline)) i2c_stop(void) asm("ass_i2c_stop");
bool __attribute__((noinline)) i2c_write(uint8_t value) asm("ass_i2c_write");
uint8_t __attribute__((noinline)) i2c_read(bool last);
#endif

void LeptonFLiR::i2cWire_beginTransmission(uint8_t addr) {
    _lastI2CError = _lastErrorCode = 0;
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

int LeptonFLiR::i2cWire_read(void) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cWire->read();
#else
    if (_readBytes > 1)
        return (int)i2c_read(_readBytes--);
    else {
        _readBytes = 0;
        int retVal = (int)i2c_read(true);
        i2c_stop();
        return retVal;
    }
#endif
}
