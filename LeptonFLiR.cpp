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

    Lepton-FLiR-Arduino - Version 0.2
*/

#include "LeptonFLiR.h"
#if (defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)) && !defined(LEPFLIR_DISABLE_SCHEDULER)
#include "Scheduler.h"
#define LEPFLIR_USE_SCHEDULER           1
#endif

#ifndef LEPFLIR_ENABLE_ALIGNED_MALLOC
static inline int roundUpVal16(int val) { return val; }
static inline uint8_t *roundUpPtr16(uint8_t *ptr) { return ptr; }
static inline uint8_t *roundUpMalloc16(int size) { return (uint8_t *)malloc((size_t)size); }
static inline uint8_t *roundUpSpiFrame16(uint8_t *spiFrame) { return spiFrame; }
#else
static inline int roundUpVal16(int val) { return ((val + 15) & -16); }
static inline uint8_t *roundUpPtr16(uint8_t *ptr) { return (ptr + 15) & ~0xF; }
static inline uint8_t *roundUpMalloc16(int size) { return (uint8_t *)malloc((size_t)(size + 15)); }
static inline uint8_t *roundUpSpiFrame16(uint8_t *spiFrame) { return roundUpPtr16(spiFrame) + 16 - 4; }
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
}

LeptonFLiR::~LeptonFLiR() {
    if (_imageData) free(_imageData);
    if (_spiFrameData) free(_spiFrameData);
    if (_telemetryData) free(_telemetryData);
}

void LeptonFLiR::init(LeptonFLiR_ImageStorageMode storageMode) {
    _storageMode = (LeptonFLiR_ImageStorageMode)constrain((int)storageMode, 0, (int)LeptonFLiR_ImageStorageMode_Count - 1);

    pinMode(_spiCSPin, OUTPUT);
    digitalWrite(_spiCSPin, HIGH);

    _imageData = roundUpMalloc16(((getImageHeight() - 1) * getImagePitch()) + (getImageWidth() * getImageBpp()));
    _spiFrameData = roundUpMalloc16(getImageDivFactor() * roundUpVal16(LEP_SPI_FRAME_SIZE));

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  SPISettings: SPIPortSpeed: ");
    for (int divisor = 2; divisor <= 128; divisor *= 2) {
        if ((double)F_CPU / divisor <= 20000000.0 + DBL_EPSILON) {
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
    return roundUpPtr16(_imageData);
}

uint8_t *LeptonFLiR::getImageDataRow(int row) {
    return roundUpPtr16(_imageData) + (getImagePitch() * row);
}

uint8_t *LeptonFLiR::getImageDataRowCol(int row, int col) {
    return roundUpPtr16(_imageData) + (getImagePitch() * row) + (getImageBpp() * col);
}

uint8_t *LeptonFLiR::getSPIFrameDataRow(int row) {
    return roundUpSpiFrame16(_spiFrameData) + (roundUpMalloc16(LEP_SPI_FRAME_SIZE) * row);
}

void setAGCEnabled(bool enabled) {

}

bool getAGCEnabled() {
    // TODO: AGC enabled accessor.
    return false;
}

void LeptonFLiR::setTelemetryEnabled(bool enabled) {
    // TODO: Telemetry mode mutator.
    if (mode) {
        if (!_telemetryData) {
            _telemetryData = malloc(LEP_SPI_FRAME_SIZE);
            memset(_telemetryData, 0, LEP_SPI_FRAME_SIZE);
        }
    }
    else {
        if (_telemetryData) {
            free(_telemetryData); _telemetryData = NULL;
        }
    }
}

bool LeptonFLiR::getTelemetryEnabled() {
    // TODO: Telemetry enabled accessor.
    return false;
}

uint16_t *LeptonFLiR::getTelemetryData() {
    return (uint16_t *)_telemetryData;
}

uint16_t LeptonFLiR::commandCode(int commandID, int commandType) {
    return (commandID & LEP_I2C_COMMAND_MODULE_ID_BIT_MASK) | (commandID & LEP_I2C_COMMAND_ID_BIT_MASK) | (commandType & LEP_I2C_COMMAND_TYPE_BIT_MASK);
}

bool LeptonFLiR::sendCommand(uint16_t commandCode) {
    return writeRegister(LEP_I2C_COMMAND_REG, &commandCode, 1);
}

bool LeptonFLiR::sendCommand(uint16_t commandCode, uint16_t value) {
    uint16_t buffer[3] = { commandCode, 1, value };
    return writeRegister(LEP_I2C_COMMAND_REG, buffer, 3);
}

bool LeptonFLiR::sendCommand(uint16_t commandCode, unsigned long value) {
    uint16_t buffer[4] = { commandCode, 2, (value >> 16) & 0xffff, value & 0xffff };
    return writeRegister(LEP_I2C_COMMAND_REG, buffer, 4);
}

bool LeptonFLiR::sendCommand(uint16_t commandCode, uint16_t *dataBuffer, int length) {
    uint16_t buffer[2] = { commandCode, length };
    return writeRegister(LEP_I2C_COMMAND_REG, buffer, 2, dataBuffer, length);
}

bool LeptonFLiR::receiveCommand(uint16_t commandCode, uint16_t *value) {
    uint16_t buffer[2] = { commandCode, 0 };
    if (!writeRegister(LEP_I2C_COMMAND_REG, buffer, 2))
        return false;

    uint16_t respLength;
    if (!readRegister(LEP_I2C_DATA_LENGTH_REG, 1, &respLength, 1) || respLength != 1)
        return false;

    return readRegister(1, value, 1);
}

bool LeptonFLiR::receiveCommand(uint16_t commandCode, unsigned long *value) {
    uint16_t buffer[2] = { commandCode, 0 };
    if (!writeRegister(LEP_I2C_COMMAND_REG, buffer, 2))
        return false;

    uint16_t respLength;
    if (!readRegister(LEP_I2C_DATA_LENGTH_REG, 1, &respLength, 1) || respLength != 2)
        return false;

    return readRegister(2, value, 2);
}

bool LeptonFLiR::receiveCommand(uint16_t commandCode, int expectedLength, uint16_t *respBuffer, int maxLength) {
    uint16_t buffer[2] = { commandCode, 0 };
    if (!writeRegister(LEP_I2C_COMMAND_REG, buffer, 2))
        return false;

    uint16_t respLength;
    if (!readRegister(LEP_I2C_DATA_LENGTH_REG, 1, &respLength, 1))
        return false;

    return readRegister(respLength, respBuffer, maxLength);
}

bool LeptonFLiR::writeRegister(uint16_t regAddress, uint16_t *dataBuffer, int length) {
    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);

    i2cWire_write(HighByte(regAddress));
    i2cWire_write(LowByte(regAddress));
    
    while (length-- > 0) {
        i2cWire_write(HighByte(*dataBuffer));
        i2cWire_write(LowByte(*dataBuffer++));
    }

    return i2cWire_endTransmission() == 0;
}

bool LeptonFLiR::writeRegister(uint16_t regAddress, uint16_t *dataBuffer1, int length1, uint16_t *dataBuffer2, int length2) {
    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);

    i2cWire_write(HighByte(regAddress));
    i2cWire_write(LowByte(regAddress));

    while (length1-- > 0) {
        i2cWire_write(HighByte(*dataBuffer1));
        i2cWire_write(LowByte(*dataBuffer1++));
    }

    while (length2-- > 0) {
        i2cWire_write(HighByte(*dataBuffer2));
        i2cWire_write(LowByte(*dataBuffer2++));
    }

    return i2cWire_endTransmission() == 0;
}

bool LeptonFLiR::readRegister(uint16_t regAddress, int expectedLength, uint16_t *respBuffer, int maxLength) {
    if (!writeRegister(regAddress, NULL, 0))
        return false;
    return readRegister(expectedLength, respBuffer, maxLength);
}

bool LeptonFLiR::readRegister(int expectedLength, uint16_t *respBuffer, int maxLength) {
    uint8_t readBytes = i2cWire_requestFrom(LEP_I2C_DEVICE_ADDRESS, expectedLength << 1);

}










#ifdef LEPFLIR_USE_SOFTWARE_I2C
bool __attribute__((noinline)) i2c_start(uint8_t addr);
void __attribute__((noinline)) i2c_stop(void) asm("ass_i2c_stop");
bool __attribute__((noinline)) i2c_write(uint8_t value) asm("ass_i2c_write");
uint8_t __attribute__((noinline)) i2c_read(bool last);
#endif

void LeptonFLiR::i2cWire_beginTransmission(uint8_t addr) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    _i2cWire->beginTransmission(addr);
#else
    i2c_start(addr);
#endif
}

uint8_t LeptonFLiR::i2cWire_endTransmission(void) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cWire->endTransmission();
#else
    i2c_stop();
    return 0;
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












// Old stuff beyond this point

const uint8_t flirCSEnablePin = 49;
uint16_t flirImage[60][80];

void flirLeptonCommand(uint16_t commandID, uint8_t commandType) {
    uint8_t error;
    Wire.beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    Wire.write(0x00);
    Wire.write(0x04);

    if (commandID & LEP_OEM_MODULE_BASE == LEP_OEM_MODULE_BASE)
        commandID = 0x4800 | (commandID & 0x00ff);

    Wire.write((commandID >> 8) & 0xff);
    Wire.write(((commandID & 0xfc) | (commandType & 0x03)) & 0xff);

    error = Wire.endTransmission();
    if (error != 0) {
        Serial.print("error=");
        Serial.println(error);
    }
}

void flirEnableAGC() {
    uint8_t error;
    Wire.beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    Wire.write(0x00);
    Wire.write(0x04);

    Wire.write(0x01);
    Wire.write(0x01);

    Wire.write(0x00);
    Wire.write(0x02);

    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x01);

    error = Wire.endTransmission();
    if (error != 0) {
        Serial.print("error=");
        Serial.println(error);
    }
}

void flirSetReg(uint16_t reg) {
    uint8_t error;
    Wire.beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    Wire.write((reg >> 8) & 0xff);
    Wire.write(reg & 0xff);

    error = Wire.endTransmission();
    if (error != 0) {
        Serial.print("error=");
        Serial.println(error);
    }
}

//Status reg 15:8 Error Code  7:3 Reserved 2:Boot Status 1:Boot Mode 0:busy
uint16_t flirReadReg(uint16_t reg) {
    uint16_t reading;
    flirSetReg(reg);

    Wire.requestFrom(LEP_I2C_DEVICE_ADDRESS, 2);

    reading = Wire.read() << 8;
    reading |= Wire.read();

    Serial.print("reg:");
    Serial.print(reg);
    Serial.print("==0x");
    Serial.print(reading, HEX);
    Serial.print(" binary:");
    Serial.println(reading, BIN);
    return reading;
}

int flirReadData() {
    uint16_t data;
    uint16_t payloadLength;

    while (flirReadReg(0x2) & 0x01) {
        Serial.println("busy");
    }

    payloadLength = flirReadReg(0x6);
    Serial.print("payloadLength=");
    Serial.println(payloadLength);

    Wire.requestFrom(LEP_I2C_DEVICE_ADDRESS, payloadLength);
    //flirSetReg(0x08); // TODO: WTF is this even doing? Why is it here? It came from GitHub.
    for (int i = 0; i < (payloadLength / 2); ++i) {
        data = Wire.read() << 8;
        data |= Wire.read();
        Serial.println(data, HEX);
    }
}

void demoFlirReading() {
    while (flirReadReg(0x2) & 0x01) {
        Serial.println("busy");
    }

    Serial.println("SYS Camera Status");
    flirLeptonCommand(LEP_CID_SYS_CAM_STATUS, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS Customer Serial Number");
    flirLeptonCommand(LEP_CID_SYS_CUST_SERIAL_NUMBER, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS FLiR Serial Number");
    flirLeptonCommand(LEP_CID_SYS_FLIR_SERIAL_NUMBER, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS Camera Uptime");
    flirLeptonCommand(LEP_CID_SYS_CAM_UPTIME, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS Aux Temperature Kelvin");
    flirLeptonCommand(LEP_CID_SYS_AUX_TEMPERATURE_KELVIN, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS Fpa Temperature Kelvin");
    flirLeptonCommand(LEP_CID_SYS_FPA_TEMPERATURE_KELVIN, FLIR_CMD_GET);
    flirReadData();

    Serial.println("AGC Enable State");
    flirLeptonCommand(LEP_CID_AGC_ENABLE_STATE, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS Telemetry Enable State");
    flirLeptonCommand(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, FLIR_CMD_GET);
    flirReadData();

    /*Serial.println("OEM Chip Mask Revision");
    flirLeptonCommand(LEP_CID_OEM_CHIP_MASK_REVISION, FLIR_CMD_GET);
    flirReadData();

    Serial.println("OEM Camera Software Revision");
    flirLeptonCommand(LEP_CID_OEM_CAM_SOFTWARE_REVISION, FLIR_CMD_GET);
    flirReadData();*/
}

void flirReadLeptonFrame(int fillRow) {
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));

    if (fillRow >= 0 && fillRow < 60) {
        uint8_t packetID;
        uint8_t packetRow;
        uint16_t packetCRC;

        //REG_PIOB_ODSR &= ~0x01;
        digitalWrite(flirCSEnablePin, LOW);

        do {
            packetID = SPI.transfer(0x00);
            packetRow = SPI.transfer(0x00);
            packetCRC = SPI.transfer(0x00) << 8;
            packetCRC |= SPI.transfer(0x00);

            if (packetID & 0x0f == 0x0f)
                delay(200); // spec says 185, doing 200 to ensure timeout
        } while (packetID & 0x0f == 0x0f);

        // CRC check polynominal: x^16 + x^12 + x^5 + x^0
        // The CRC is calculated over the entire packet, including the ID and CRC fields.
        // However, the four most-significant bits of the ID and all sixteen bits of the
        // CRC are set to zero for calculation of the CRC. There is no requirement for the
        // host to verify the CRC. However, if the host does find a CRC mismatch, it is
        // recommended to re-synchronize the VoSPI stream to prevent potential misalignment.

        Serial.print("{ packetID=");
        Serial.print(packetID, HEX);
        Serial.print(", packetRow=");
        Serial.print(packetRow, HEX);
        Serial.print(", packetCRC=");
        Serial.print(packetCRC, HEX);
        Serial.println(" }");

        for (int i = 0; i < 80; ++i) {
            flirImage[fillRow][i] = SPI.transfer(0x00) << 8;
            flirImage[fillRow][i] |= SPI.transfer(0x00);
        }

        digitalWrite(flirCSEnablePin, HIGH);
        //REG_PIOB_ODSR |= 0x01;
    }

    SPI.endTransaction();
}
