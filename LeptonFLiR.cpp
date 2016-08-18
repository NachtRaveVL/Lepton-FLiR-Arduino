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

    Lepton-FLiR-Arduino - Version 0.7
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
    _spiFrameData = roundUpMalloc16(getSPIFrameLines() * roundUpVal16(LEP_SPI_FRAME_SIZE));

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

uint8_t *LeptonFLiR::getImageData() {
    return !_isReadingNextFrame ? roundUpPtr16(_imageData) : NULL;
}

uint8_t *LeptonFLiR::getImageDataRow(int row) {
    return !_isReadingNextFrame ? (roundUpPtr16(_imageData) + (getImagePitch() * row)) : NULL;
}

uint8_t *LeptonFLiR::_getImageDataRow(int row) {
    return roundUpPtr16(_imageData) + (getImagePitch() * row);
}

uint8_t *LeptonFLiR::getTelemetryData() {
    // Don't let user have access to telemetry data if it hasn't been filled yet (that is, ID is discard packet)
    return !_isReadingNextFrame && _telemetryData && _telemetryData[0] != 0x0F ? _telemetryData : NULL;
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

static void SPI_transfer(uint8_t *buffer, size_t count) {
    while (count-- > 0)
        *buffer++ = (uint8_t)SPI.transfer((uint8_t)0x00);
}

bool LeptonFLiR::readNextFrame() {
    if (!_isReadingNextFrame) {
        _isReadingNextFrame = true;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.println("LeptonFLiR::readNextFrame");
#endif

        bool agc8Enabled, telemetryEnabled, cameraBooted; LEP_SYS_TELEMETRY_LOCATION telemetryLocation;
        {   uint16_t value;

            receiveCommand(commandCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &value);
            agc8Enabled = value;

            if (agc8Enabled) {
                receiveCommand(commandCode(LEP_CID_AGC_HEQ_SCALE_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &value);
                agc8Enabled = (value == (uint16_t)LEP_AGC_SCALE_TO_8_BITS);
            }
            
            receiveCommand(commandCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &value);
            telemetryEnabled = value;
            
            receiveCommand(commandCode(LEP_CID_SYS_TELEMETRY_LOCATION, LEP_I2C_COMMAND_TYPE_GET), &value);
            telemetryLocation = (LEP_SYS_TELEMETRY_LOCATION)value;

            readRegister(LEP_I2C_STATUS_REG, &value, 1, 1);
            cameraBooted = (value & LEP_I2C_STATUS_BOOT_MODE_BIT_MASK) && (value & LEP_I2C_STATUS_BOOT_STATUS_BIT_MASK);
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
        Serial.print("  LeptonFLiR::readNextFrame AGC8: ");
        Serial.print(agc8Enabled ? "enabled" : "disabled");
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
        uint_fast8_t spiRows = getSPIFrameLines();
        uint_fast8_t currSpiRow = 0;
        uint_fast8_t teleRows = (telemetryEnabled * 3);
        uint_fast8_t currTeleRow = 0;
        uint_fast8_t framesSkipped = 0;
        uint_fast16_t packetsRead = 0;
        uint8_t *spiFrame = getSPIFrameDataRow(currSpiRow);
        bool packetHeaderRead = true;

        SPI.beginTransaction(_spiSettings);

        digitalWrite(_spiCSPin, LOW);
        delayTimeout(185);
        SPI_transfer(spiFrame, 4);
        
        while (currImgRow < imgRows || currTeleRow < teleRows) {
            if (!packetHeaderRead) {
                spiFrame = getSPIFrameDataRow(currSpiRow);
                ++packetsRead;
                digitalWrite(_spiCSPin, LOW);
                SPI_transfer(spiFrame, 4);
            }
            else
                packetHeaderRead = false;

            if (spiFrame[0] == 0x00 && spiFrame[1] == readLines) { // Image packet
                uint8_t *pxlData = (_storageMode == LeptonFLiR_ImageStorageMode_80x60_16bpp ? _getImageDataRow(readLines) : &spiFrame[4]);

                SPI_transfer(pxlData, LEP_SPI_FRAME_SIZE - 4);
                digitalWrite(_spiCSPin, HIGH);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                Serial.println("    LeptonFLiR::readNextFrame VoSPI Image Packet:");
                Serial.print("      ");  printSPIFrame(spiFrame, pxlData);
#endif

                ++readLines; ++currSpiRow;
            }
            else if ((spiFrame[0] & 0x0F) != 0x0F && teleRows && currTeleRow < 3 &&
                ((telemetryLocation == LEP_TELEMETRY_LOCATION_HEADER && readLines == 0) ||
                (telemetryLocation == LEP_TELEMETRY_LOCATION_FOOTER && readLines == 60))) { // Telemetry packet
                if (currTeleRow == 0) {
                    SPI_transfer(&_telemetryData[4], LEP_SPI_FRAME_SIZE - 4);
                    digitalWrite(_spiCSPin, HIGH);
                    memcpy(_telemetryData, spiFrame, 4);
                    //wroteTeleData = true;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                    Serial.println("    LeptonFLiR::readNextFrame VoSPI Telemetry(A) Packet:");
                    Serial.print("      ");  printSPIFrame(_telemetryData, &_telemetryData[4]);
#endif
                }
                else {
                    SPI_transfer(&spiFrame[4], LEP_SPI_FRAME_SIZE - 4);
                    digitalWrite(_spiCSPin, HIGH);

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
                SPI_transfer(&spiFrame[4], LEP_SPI_FRAME_SIZE - 4);
                digitalWrite(_spiCSPin, HIGH);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                Serial.println("    LeptonFLiR::readNextFrame VoSPI Discard Packet:");
                Serial.print("      ");  printSPIFrame(spiFrame, &spiFrame[4]);
#endif

                if ((spiFrame[0] & 0x0F) == 0x0F)
                    delayTimeout(185);

                spiFrame = getSPIFrameDataRow(currSpiRow);
                packetHeaderRead = true;
                uint_fast8_t triesLeft = 120;
                
                while (triesLeft > 0) {
                    ++packetsRead;
                    digitalWrite(_spiCSPin, LOW);
                    SPI_transfer(spiFrame, 4);

                    if ((spiFrame[0] & 0x0F) != 0x0F) {
                        if ((spiFrame[0] == 0x00 && spiFrame[1] == readLines) ||
                            (teleRows && readLines == 60 && spiFrame[0] > 0x00 && telemetryLocation == LEP_TELEMETRY_LOCATION_FOOTER)) {
                            break;
                        }
                        else if ((spiFrame[0] == 0x00 && spiFrame[1] == 0) ||
                            (teleRows && spiFrame[0] > 0x00 && telemetryLocation == LEP_TELEMETRY_LOCATION_HEADER)) {
                            
                            if (++framesSkipped >= 5) {
                                SPI_transfer(&spiFrame[4], LEP_SPI_FRAME_SIZE - 4);
                                digitalWrite(_spiCSPin, HIGH);
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                                Serial.println("  LeptonFLiR::readNextFrame Maximum frame skip reached. Aborting.");
#endif
                                SPI.endTransaction();
                                _isReadingNextFrame = false;
                                return false;
                            }

                            readLines = currImgRow = currSpiRow = currTeleRow = 0;
                            break;
                        }
                    }

                    SPI_transfer(&spiFrame[4], LEP_SPI_FRAME_SIZE - 4);
                    digitalWrite(_spiCSPin, HIGH);
                    --triesLeft;
                }

                if (triesLeft == 0) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
                    Serial.println("  LeptonFLiR::readNextFrame Maximum resync retries reached. Aborting.");
#endif
                    SPI.endTransaction();
                    _isReadingNextFrame = false;
                    return false;
                }
            }

            // Write out to frame
            if (currSpiRow == spiRows && (!teleRows || currTeleRow > 1)) {
                if (_storageMode != LeptonFLiR_ImageStorageMode_80x60_16bpp) {
                    spiFrame = getSPIFrameDataRow(0) + 4;
                    uint8_t *pxlData = _getImageDataRow(currImgRow);

                    uint_fast8_t imgWidth = getImageWidth();
                    uint_fast8_t imgBpp = getImageBpp();

                    uint_fast32_t divisor = (spiRows * spiRows) * (!agc8Enabled && imgBpp == 1 ? 64 : 1);
                    uint_fast32_t clamp = (!agc8Enabled && imgBpp == 2 ? 0x3FFF : 0x00FF);

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

    uint16_t enabled;
    receiveCommand(commandCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::setAGCPolicy(LEP_AGC_POLICY policy) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("LeptonFLiR::setAGCPolicy policy: ");
    Serial.println(policy);
#endif

    sendCommand(commandCode(LEP_CID_AGC_POLICY, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)policy);
}

LEP_AGC_POLICY LeptonFLiR::getAGCPolicy() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getAGCPolicy");
#endif

    uint16_t policy;
    receiveCommand(commandCode(LEP_CID_AGC_POLICY, LEP_I2C_COMMAND_TYPE_GET), &policy);
    return (LEP_AGC_POLICY)policy;
}

void LeptonFLiR::setAGCCalcEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("LeptonFLiR::setAGCCalcEnabled enabled: ");
    Serial.println(enabled);
#endif

    sendCommand(commandCode(LEP_CID_AGC_CALC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::getAGCCalcEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getAGCCalcEnabled");
#endif

    uint16_t enabled;
    receiveCommand(commandCode(LEP_CID_AGC_CALC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

LEP_SYS_CAM_STATUS LeptonFLiR::getSysCameraStatus() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getSysCameraStatus");
#endif

    LEP_SYS_CAM_STATUS status;
    receiveCommand(commandCode(LEP_CID_SYS_CAM_STATUS, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)&status, sizeof(LEP_SYS_CAM_STATUS) >> 1);
    return status;
}

static void uint8ToHexString(uint8_t value, char *buffer) {
    uint8_t highNibble = value / 16;
    uint8_t lowNibble = value % 16;
    if (highNibble < 10) buffer[0] = '0' + highNibble;
    else buffer[0] = 'A' + (highNibble - 10);
    if (lowNibble < 10) buffer[1] = '0' + lowNibble;
    else buffer[1] = 'A' + (lowNibble - 10);
}

void LeptonFLiR::getSysFlirSerialNumber(char *buffer, int maxLength) {
    if (!buffer || maxLength < 16) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getSysFlirSerialNumber");
#endif

    uint16_t innerBuffer[4];
    receiveCommand(commandCode(LEP_CID_SYS_FLIR_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), innerBuffer, 4);
    
    for (int i = 0; i < 4; ++i) {
        uint8ToHexString(highByte(innerBuffer[i]), &buffer[(i * 4) + 0]);
        uint8ToHexString(lowByte(innerBuffer[i]), &buffer[(i * 4) + 2]);
    }

    if (maxLength > 16)
        buffer[16] = '\0';
}

void LeptonFLiR::getSysCustomerSerialNumber(char *buffer, int maxLength) {
    if (!buffer || maxLength < 32) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getSysCustomerSerialNumber");
#endif

    receiveCommand(commandCode(LEP_CID_SYS_CUST_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)buffer, 16);

    if (maxLength > 32)
        buffer[32] = '\0';
}

uint32_t LeptonFLiR::getSysCameraUptime() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getSysCameraUptime");
#endif

    uint32_t uptime;
    receiveCommand(commandCode(LEP_CID_SYS_CAM_UPTIME, LEP_I2C_COMMAND_TYPE_GET), &uptime);
    return uptime;
}

float lepK100ToCelsius(uint16_t lepK100) {
    float kelvins = (lepK100 / 100.0) + ((lepK100 % 100) * 0.01);
    return kelvins - 273.15;
}

float lepK100ToFarenheit(uint16_t lepK100) {
    float kelvins = (lepK100 / 100.0) + ((lepK100 % 100) * 0.01);
    return round((((kelvins * 9.0) / 5.0) - 459.67) * 100.0) / 100.0;
}

uint16_t celsiusToLepK100(float celsius) {
    float kelvins = celsius + 273.15;
    return (uint16_t)round(kelvins * 100.0);
}

uint16_t farenheitToLepK100(float farenheit) {
    float kelvins = ((farenheit + 459.67) * 5.0) / 9.0;
    return (uint16_t)round(kelvins * 100.0);
}

uint16_t LeptonFLiR::getSysAuxTemperature() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getSysAuxTemperature");
#endif

    uint16_t lepK100;
    receiveCommand(commandCode(LEP_CID_SYS_AUX_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), &lepK100);
    return lepK100;
}
uint16_t LeptonFLiR::getSysFPATemperature() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getSysFPATemperature");
#endif

    uint16_t lepK100;
    receiveCommand(commandCode(LEP_CID_SYS_FPA_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), &lepK100);
    return lepK100;
}

void LeptonFLiR::setSysTelemetryEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("LeptonFLiR::setSysTelemetryEnabled enabled: ");
    Serial.println(enabled);
#endif

    sendCommand(commandCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
    
    if (!_lastI2CError && !_lastErrorCode) {
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
}

bool LeptonFLiR::getSysTelemetryEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getSysTelemetryEnabled");
#endif

    uint16_t enabled;
    receiveCommand(commandCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::setVidPolarity(LEP_VID_POLARITY polarity) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("LeptonFLiR::setVidPolarity polarity: ");
    Serial.println(polarity);
#endif

    sendCommand(commandCode(LEP_CID_VID_POLARITY_SELECT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)polarity);
}

LEP_VID_POLARITY LeptonFLiR::getVidPolarity() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getVidPolarity");
#endif

    uint16_t polarity;
    receiveCommand(commandCode(LEP_CID_VID_POLARITY_SELECT, LEP_I2C_COMMAND_TYPE_GET), &polarity);
    return (LEP_VID_POLARITY)polarity;
}

void LeptonFLiR::setVidPseudoColorLUT(LEP_VID_PCOLOR_LUT lut) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("LeptonFLiR::setVidPseudoColorLUT lut: ");
    Serial.println(lut);
#endif

    sendCommand(commandCode(LEP_CID_VID_LUT_SELECT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)lut);
}

LEP_VID_PCOLOR_LUT LeptonFLiR::getVidPseudoColorLUT() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::getVidPseudoColorLUT");
#endif

    uint16_t lut;
    receiveCommand(commandCode(LEP_CID_VID_LUT_SELECT, LEP_I2C_COMMAND_TYPE_GET), &lut);
    return (LEP_VID_PCOLOR_LUT)lut;
}

void LeptonFLiR::setVidFocusCalcEnabled(bool enabled) { // LEP_CID_VID_FOCUS_CALC_ENABLE
}

bool LeptonFLiR::getVidFocusCalcEnabled() {
}

void LeptonFLiR::setVidFreezeEnabled(bool enabled) { // LEP_CID_VID_FREEZE_ENABLE
}

bool LeptonFLiR::getVidFreezeEnabled() {
}

#ifndef PCA9685_EXCLUDE_EXT_I2C_FUNCS

void setAGCHistogramRegion(LEP_AGC_HISTOGRAM_ROI region) { return; } // LEP_CID_AGC_ROI
LEP_AGC_HISTOGRAM_ROI getAGCHistogramRegion() { return LEP_AGC_HISTOGRAM_ROI(); }

LEP_AGC_HISTOGRAM_STATISTICS LeptonFLiR::getAGCHistogramStatistics() { return LEP_AGC_HISTOGRAM_STATISTICS(); } // LEP_CID_AGC_STATISTICS

void LeptonFLiR::setAGCHistogramClipPercent(uint16_t value) { return; } // LEP_CID_AGC_HISTOGRAM_CLIP_PERCENT
uint16_t LeptonFLiR::getAGCHistogramClipPercent() { return 0; } // ???

void LeptonFLiR::setAGCHistogramTailSize(uint16_t value) { return; } // LEP_CID_AGC_HISTOGRAM_TAIL_SIZE
uint16_t LeptonFLiR::getAGCHistogramTailSize() { return 0; } // ???

void LeptonFLiR::setAGCLinearMaxGain(uint16_t value) { return; } // LEP_CID_AGC_LINEAR_MAX_GAIN
uint16_t LeptonFLiR::getAGCLinearMaxGain() { return 0; } // ???

void LeptonFLiR::setAGCLinearMidpoint(uint16_t value) { return; } // LEP_CID_AGC_LINEAR_MIDPOINT
uint16_t LeptonFLiR::getAGCLinearMidpoint() { return 0; } // ???

void LeptonFLiR::setAGCLinearDampeningFactor(uint16_t value) { return; } // LEP_CID_AGC_LINEAR_DAMPENING_FACTOR
uint16_t LeptonFLiR::getAGCLinearDampeningFactor() { return 0; } // ???

void LeptonFLiR::setAGCHEQDampeningFactor(uint16_t value) { return; } // LEP_CID_AGC_HEQ_DAMPENING_FACTOR
uint16_t LeptonFLiR::getAGCHEQDampeningFactor() { return 0; }

void LeptonFLiR::setAGCHEQMaxGain(uint16_t value) { return; } // LEP_CID_AGC_HEQ_MAX_GAIN
uint16_t LeptonFLiR::getAGCHEQMaxGain() { return 0; } // ???

void LeptonFLiR::setAGCHEQClipLimitHigh(uint16_t value) { return; } // LEP_CID_AGC_HEQ_CLIP_LIMIT_HIGH
uint16_t LeptonFLiR::getAGCHEQClipLimitHigh() { return 0; }

void LeptonFLiR::setAGCHEQClipLimitLow(uint16_t value) { return; } // LEP_CID_AGC_HEQ_CLIP_LIMIT_LOW
uint16_t LeptonFLiR::getAGCHEQClipLimitLow() { return 0; }

void LeptonFLiR::setAGCHEQBinExtension(uint16_t value) { return; } // LEP_CID_AGC_HEQ_BIN_EXTENSION
uint16_t LeptonFLiR::getAGCHEQBinExtension() { return 0; } // ???

void LeptonFLiR::setAGCHEQMidpoint(uint16_t value) { return; } // LEP_CID_AGC_HEQ_MIDPOINT
uint16_t LeptonFLiR::getAGCHEQMidpoint() { return 0; } // ???

void LeptonFLiR::setAGCHEQEmptyCounts(uint16_t value) { return; } // LEP_CID_AGC_HEQ_EMPTY_COUNTS
uint16_t LeptonFLiR::getAGCHEQEmptyCounts() { return 0; }

void LeptonFLiR::setAGCHEQNormalizationFactor(uint16_t value) { return; } // LEP_CID_AGC_HEQ_NORMALIZATION_FACTOR
uint16_t LeptonFLiR::getAGCHEQNormalizationFactor() { return 0; } // ???

void LeptonFLiR::setAGCHEQScaleFactor(LEP_AGC_HEQ_SCALE_FACTOR factor) { return; } // LEP_CID_AGC_HEQ_SCALE_FACTOR
LEP_AGC_HEQ_SCALE_FACTOR LeptonFLiR::getAGCHEQScaleFactor() { return (LEP_AGC_HEQ_SCALE_FACTOR)0; }

void LeptonFLiR::runSysPingCamera() { return; } // LEP_CID_SYS_PING

void LeptonFLiR::setSysTelemetryLocation(LEP_SYS_TELEMETRY_LOCATION location) { return; } // LEP_CID_SYS_TELEMETRY_LOCATION
LEP_SYS_TELEMETRY_LOCATION LeptonFLiR::getSysTelemetryLocation() { return (LEP_SYS_TELEMETRY_LOCATION)0; }

void LeptonFLiR::runSysExecuteFrameAverage() { return; } // LEP_CID_SYS_EXECTUE_FRAME_AVERAGE ??? (maybe get or set?)

void LeptonFLiR::setSysNumFramesToAverage(LEP_SYS_FRAME_AVERAGE average) { return; } // LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE
LEP_SYS_FRAME_AVERAGE LeptonFLiR::getSysNumFramesToAverage() { return (LEP_SYS_FRAME_AVERAGE)0; }

LEP_SYS_SCENE_STATISTICS LeptonFLiR::getSysSceneStatistics() { return LEP_SYS_SCENE_STATISTICS(); } // LEP_CID_SYS_SCENE_STATISTICS

void LeptonFLiR::setSysSceneRegion(LEP_SYS_SCENE_ROI region) { return; } // LEP_CID_SYS_SCENE_ROI
LEP_SYS_SCENE_ROI LeptonFLiR::getSysSceneRegion() { return LEP_SYS_SCENE_ROI(); }

uint16_t LeptonFLiR::getSysThermalShutdownCount() { return 0; } // LEP_CID_SYS_THERMAL_SHUTDOWN_COUNT

void LeptonFLiR::setSysShutterPosition(LEP_SYS_SHUTTER_POSITION position) { return; } // LEP_CID_SYS_SHUTTER_POSITION
LEP_SYS_SHUTTER_POSITION LeptonFLiR::getSysShutterPosition() { return (LEP_SYS_SHUTTER_POSITION)0; }

void LeptonFLiR::setSysFFCShutterMode(LEP_SYS_FFC_SHUTTER_MODE mode) { return; } // LEP_CID_SYS_FFC_SHUTTER_MODE
LEP_SYS_FFC_SHUTTER_MODE LeptonFLiR::getSysFFCShutterMode() { return LEP_SYS_FFC_SHUTTER_MODE(); }

void LeptonFLiR::runSysFlatFieldCorrection() { return; } // LEP_CID_SYS_RUN_FFC

LEP_SYS_FFC_STATUS LeptonFLiR::getSysFlatFieldCorrectionStatus() { return (LEP_SYS_FFC_STATUS)0; } // LEP_CID_SYS_FFC_STATUS

void LeptonFLiR::setVidUserColorLUT(LEP_VID_LUT_BUFFER *table) { return; } // LEP_CID_VID_LUT_TRANSFER
void LeptonFLiR::getVidUserColorLUT(LEP_VID_LUT_BUFFER *table) { return; }

void LeptonFLiR::setVidFocusRegion(LEP_VID_FOCUS_ROI region) { return; } // LEP_CID_VID_FOCUS_ROI
LEP_VID_FOCUS_ROI LeptonFLiR::getVidFocusRegion() { return LEP_VID_FOCUS_ROI(); }

void LeptonFLiR::setVidFocusThreshold(uint32_t threshold) { return; } // LEP_CID_VID_FOCUS_THRESHOLD
uint32_t LeptonFLiR::getVidFocusThreshold() { return 0; }

uint32_t LeptonFLiR::getVidFocusMetric() { return 0; } // LEP_CID_VID_FOCUS_METRIC

void LeptonFLiR::setVidSBNUCEnabled(bool enabled) { return; } // LEP_CID_VID_SBNUC_ENABLE
bool LeptonFLiR::getVidSBNUSEnabled() { return 0; } // ???

void LeptonFLiR::setVidGamma(uint32_t value) { return; } // LEP_CID_VID_GAMMA_SELECT
uint32_t LeptonFLiR::getVidGamma() { return 0; } // ???

#endif

uint8_t LeptonFLiR::getLastI2CError() {
    return _lastI2CError;
}

LEP_RESULT LeptonFLiR::getLastErrorCode() {
    return (LEP_RESULT)(*((int8_t *)((void *)(&_lastErrorCode))));
}

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

void LeptonFLiR::printModuleInfo() {
    uint16_t data[32];

    Serial.println("SYS Camera Status:");
    receiveCommand(commandCode(LEP_CID_SYS_CAM_STATUS, LEP_I2C_COMMAND_TYPE_GET), data, 32);

    Serial.println("SYS Customer Serial Number:");
    receiveCommand(commandCode(LEP_CID_SYS_CUST_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), data, 32);

    Serial.println("SYS FLiR Serial Number:");
    receiveCommand(commandCode(LEP_CID_SYS_FLIR_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), data, 32);

    Serial.println("SYS Camera Uptime:");
    receiveCommand(commandCode(LEP_CID_SYS_CAM_UPTIME, LEP_I2C_COMMAND_TYPE_GET), data, 32);
    
    Serial.println("SYS Aux Temperature Kelvin:");
    receiveCommand(commandCode(LEP_CID_SYS_AUX_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), data, 32);

    Serial.println("SYS FPA Temperature Kelvin:");
    receiveCommand(commandCode(LEP_CID_SYS_FPA_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), data, 32);

    Serial.println("AGC Enable State:");
    receiveCommand(commandCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), data, 32);

    Serial.println("SYS Telemetry Enable State:");
    receiveCommand(commandCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), data, 32);
}

#endif

bool LeptonFLiR::waitCommandBegin(int timeout) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("    LeptonFLiR::waitCommandBegin timeout: ");
    Serial.println(timeout);
#endif

    uint16_t status;
    readRegister(LEP_I2C_STATUS_REG, &status, 1, 1);
    
    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK)) {
        _lastErrorCode = 0;
        return true;
    }

    unsigned long endTime = millis() + (unsigned long)timeout;

    while ((status & LEP_I2C_STATUS_BUSY_BIT_MASK) && (timeout <= 0 || millis() < endTime)) {
#ifdef LEPFLIR_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif

        readRegister(LEP_I2C_STATUS_REG, &status, 1, 1);
    }

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK)) {
        _lastErrorCode = 0;
        return true;
    }
    else {
        _lastErrorCode = LEP_TIMEOUT_ERROR;
        return false;
    }
}

bool LeptonFLiR::waitCommandFinish(int timeout) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("    LeptonFLiR::waitCommandFinish timeout: ");
    Serial.println(timeout);
#endif

    uint16_t status;
    readRegister(LEP_I2C_STATUS_REG, &status, 1, 1);
    
    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK)) {
        _lastErrorCode = (uint8_t)((status & LEP_I2C_STATUS_ERROR_CODE_BIT_MASK) >> LEP_I2C_STATUS_ERROR_CODE_BIT_SHIFT);
        return true;
    }

    unsigned long endTime = millis() + (unsigned long)timeout;

    while ((status & LEP_I2C_STATUS_BUSY_BIT_MASK) && (timeout <= 0 || millis() < endTime)) {
#ifdef LEPFLIR_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif

        readRegister(LEP_I2C_STATUS_REG, &status, 1, 1);
    }

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK)) {
        _lastErrorCode = (uint8_t)((status & LEP_I2C_STATUS_ERROR_CODE_BIT_MASK) >> LEP_I2C_STATUS_ERROR_CODE_BIT_SHIFT);
        return true;
    }
    else {
        _lastErrorCode = LEP_TIMEOUT_ERROR;
        return false;
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

    if (!waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT))
        return;

    uint16_t cmdBuffer[2] = { cmdCode, 0 };
    if (writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2) == 0)
        waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint16_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (!waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT))
        return;

    uint16_t cmdBuffer[3] = { cmdCode, (uint16_t)1, value };
    if (writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 3) == 0)
        waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint32_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (!waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT))
        return;

    uint16_t cmdBuffer[4] = { cmdCode, (uint16_t)2, (uint16_t)((value >> 16) & 0xFFFF), (uint16_t)(value & 0xFFFF) };
    if (writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 4) == 0)
        waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint16_t *dataBuffer, int dataLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (!waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT))
        return;

    int retStat;
    if (dataLength <= 16) {
        uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)dataLength };
        retStat = writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2, dataBuffer, dataLength);
    }
    else if (dataLength < LEP_I2C_DATA_BUFFER_0_LENGTH >> 1) {
        if ((retStat = writeRegister(LEP_I2C_DATA_BUFFER_0, dataBuffer, dataLength)) == 0) {
            uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)dataLength };
            retStat = writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2);
        }
    }
    else
        retStat = (_lastI2CError = 4);

    if (retStat == 0)
        waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
}

int LeptonFLiR::receiveCommand(uint16_t cmdCode, uint16_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (!waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT))
        return 0;

    uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)0 };
    if (writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2) == 0) {
        if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

            uint16_t respLength;
            if (readRegister(LEP_I2C_DATA_LENGTH_REG, &respLength, 1, 1) == 0) {

                if ((respLength == 1 && readRegister((uint16_t *)value, 1, 1) == 0) ||
                    (respLength > 1 && respLength <= 16 && readRegister(LEP_I2C_DATA_0_REG + ((respLength - 1) * 0x02), (uint16_t *)value, 1, 1) == 0) ||
                    (respLength > 16 && respLength < LEP_I2C_DATA_BUFFER_0_LENGTH >> 1 && readRegister(LEP_I2C_DATA_BUFFER_0 + ((respLength - 1) * 0x02), (uint16_t *)value, 1, 1) == 0))
                    return 1;
                else
                    _lastI2CError = 4;
            }
        }
    }

    return 0;
}

int LeptonFLiR::receiveCommand(uint16_t cmdCode, uint32_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (!waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT))
        return 0;

    uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)0 };
    if (writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2) == 0) {
        if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

            uint16_t respLength;
            if (readRegister(LEP_I2C_DATA_LENGTH_REG, &respLength, 1, 1) == 0) {

                if ((respLength == 2 && readRegister((uint16_t *)value, 2, 2) == 0) ||
                    (respLength > 1 && respLength <= 16 && readRegister(LEP_I2C_DATA_0_REG + ((respLength - 1) * 0x02), (uint16_t *)value, 2, 2) == 0) ||
                    (respLength > 16 && respLength < LEP_I2C_DATA_BUFFER_0_LENGTH >> 1 && readRegister(LEP_I2C_DATA_BUFFER_0 + ((respLength - 2) * 0x02), (uint16_t *)value, 2, 2) == 0))
                    return 2;
                else
                    _lastI2CError = 4;
            }
        }
    }

    return 0;
}

int LeptonFLiR::receiveCommand(uint16_t cmdCode, uint16_t *respBuffer, int maxLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (!waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT))
        return 0;

    uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)0 };
    if (writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2) == 0) {
        if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

            uint16_t respLength;
            if (readRegister(LEP_I2C_DATA_LENGTH_REG, &respLength, 1, 1) == 0) {

                if ((respLength > 0 && respLength <= 16 && readRegister(respBuffer, respLength, maxLength) == 0) ||
                    (respLength > 16 && respLength < LEP_I2C_DATA_BUFFER_0_LENGTH >> 1 && readRegister(LEP_I2C_DATA_BUFFER_0, respBuffer, respLength, maxLength) == 0))
                    return respLength;
                else
                    _lastI2CError = 4;
            }
        }
    }

    return 0;
}

int LeptonFLiR::sendReceiveCommand(uint16_t cmdCode, uint16_t *dataBuffer, int dataLength, uint16_t *respBuffer, int maxLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendReceiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (!waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT))
        return 0;

    int retStat;
    if (dataLength <= 16) {
        uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)dataLength };
        retStat = writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2, dataBuffer, dataLength);
    }
    else if (dataLength < LEP_I2C_DATA_BUFFER_0_LENGTH >> 1) {
        if ((retStat = writeRegister(LEP_I2C_DATA_BUFFER_0, dataBuffer, dataLength)) == 0) {
            uint16_t cmdBuffer[2] = { cmdCode, (uint16_t)dataLength };
            retStat = writeRegister(LEP_I2C_COMMAND_REG, cmdBuffer, 2);
        }
    }
    else
        retStat = (_lastI2CError = 4);

    if (retStat == 0) {
        if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

            uint16_t respLength;
            if (readRegister(LEP_I2C_DATA_LENGTH_REG, &respLength, 1, 1) == 0) {

                if ((respLength > 0 && respLength <= 16 && readRegister(respBuffer, respLength, maxLength) == 0) ||
                    (respLength > 16 && respLength < LEP_I2C_DATA_BUFFER_0_LENGTH >> 1 && readRegister(LEP_I2C_DATA_BUFFER_0, respBuffer, respLength, maxLength) == 0))
                    return respLength;
                else
                    _lastI2CError = 4;
            }
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

    return i2cWire_endTransmission();
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

    return i2cWire_endTransmission();
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

    if (i2cWire_endTransmission() == 0)
        return readRegister(respBuffer, respLength, maxLength);
    return _lastI2CError;
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

        while (wordsRead > 0 && maxLength > 0) {
            --wordsRead; --maxLength;
            *respBuffer = ((uint16_t)(i2cWire_read() & 0xFF) << 8);
            *respBuffer++ |= (uint16_t)(i2cWire_read() & 0xFF);
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

        while (wordsRead-- > 0) {
            i2cWire_read();
            i2cWire_read();
        }

        return (_lastI2CError = 0);
    }

    return (_lastI2CError = 4);
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
