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

#ifndef LeptonFLiR_H
#define LeptonFLiR_H

// Library Setup

// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
//#define LEPFLIR_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define to disable usage of the Scheduler library on SAM/SAMD architecures.
//#define LEPFLIR_DISABLE_SCHEDULER       1   // https://github.com/arduino-libraries/Scheduler

// Uncomment this define to enable 16-byte aligned memory allocations (may improve performance).
//#define LEPFLIR_ENABLE_ALIGNED_MALLOC   1

// Uncomment this define to enable debug output.
//#define LEPFLIR_ENABLE_DEBUG_OUTPUT     1

// Hookup Instructions
// Make sure to hookup SPI lines MISO, MOSI, and CLK correctly (Due, Zero, etc. often use
// pins 50=MISO, 51=MOSI, 52=CLK, 53=CS, but one can always just use the ICSP header pins
// ICSP-1=MISO, ICSP-4=MOSI, ICSP-3=CLK). The CS (chip select) pin may be any digital
// output pin, with usage being active-low. The recommended VCC supply and logic level is
// 3.3v, but 5v also seems to work. The two issolated side power pins on breakout can be
// left disconnected. The minimum SPI transfer rate is 2MHz, which means one needs at
// least a 4MHz board, but realistically a 16MHz board is likely required given the
// processing work involved. The actual SPI transfer rate selected will be the first one
// below 20MHz given the SPI clock divider (processor speed /2, /4, /8, ..., /128).

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
#include <Wire.h>
#endif
#include <SPI.h>
#include "LeptonFLiRDefs.h"

// Image storage mode affects the total memory footprint. Memory constrained boards
// should take notice to the storage requirements. Note that the Lepton FLiR delivers
// 14bpp thermal image data with AGC mode disabled and 8bpp thermal image data with AGC
// mode enabled, therefore if using AGC mode always enabled it is more memory efficient
// to use an 8bpp mode to begin with. Note that with telemetry enabled, memory cost
// incurs an additional 164 bytes for telemetry data storage.
typedef enum {
    LeptonFLiR_ImageStorageMode_80x60_16bpp,    // Full 16bpp image mode, 9600 bytes for image data, 164 bytes for read frame (9764 bytes total, 9806 bytes if aligned)
    LeptonFLiR_ImageStorageMode_80x60_8bpp,     // Full 8bpp image mode, 4800 bytes for image data, 164 bytes for read frame (4964 bytes total, 5006 bytes if aligned)
    LeptonFLiR_ImageStorageMode_40x30_16bpp,    // Halved 16bpp image mode, 2400 bytes for image data, 328 bytes for read frame (2728 bytes total, 2782 bytes if aligned)
    LeptonFLiR_ImageStorageMode_40x30_8bpp,     // Halved 8bpp image mode, 1200 bytes for image data, 328 bytes for read frame (1528 bytes total, 1814 bytes if aligned)
    LeptonFLiR_ImageStorageMode_20x15_16bpp,    // Quartered 16bpp image mode, 600 bytes for image data, 656 bytes for read frame (1256 bytes total, 1446 bytes if aligned)
    LeptonFLiR_ImageStorageMode_20x15_8bpp,     // Quartered 8bpp image mode, 300 bytes for image data, 656 bytes for read frame (956 bytes total, 1202 bytes if aligned)
    LeptonFLiR_ImageStorageMode_Count
} LeptonFLiR_ImageStorageMode;

class LeptonFLiR {
public:
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    // May use a different Wire instance than Wire. Some chipsets, such as Due/Zero/etc.,
    // have a Wire1 class instance that uses the SDA1/SCL1 lines instead.
    // Supported i2c baud rates are 100kHz, 400kHz, and 1000kHz.
    // Supported SPI baud rates are 2MHz to 20MHz.
    LeptonFLiR(TwoWire& i2cWire = Wire, uint8_t spiCSPin = 53);
#else
    // Minimum supported i2c baud rate is 100kHz, which means minimum supported processor
    // speed is 4MHz+ while running i2c standard mode. For 400kHz i2c baud rate, minimum
    // supported processor speed is 16MHz+ while running i2c fast mode.
    // Supported SPI baud rates are 2MHz to 20MHz.
    LeptonFLiR(uint8_t spiCSPin = 53);
#endif
    ~LeptonFLiR();

    // Called in setup()
    void init(LeptonFLiR_ImageStorageMode storageMode = LeptonFLiR_ImageStorageMode_80x60_16bpp);

    LeptonFLiR_ImageStorageMode getImageStorageMode();
    int getImageWidth();
    int getImageHeight();
    int getImageBpp(); // Bytes per pixel
    int getImagePitch(); // Bytes per row
    uint8_t *getImageData();
    uint8_t *getImageDataRow(int row);
    uint8_t *getImageDataRowCol(int row, int col);

    void setAGCEnabled(bool enabled);
    bool getAGCEnabled();

    void setTelemetryEnabled(bool enabled);
    bool getTelemetryEnabled();
    uint16_t *getTelemetryData();

    uint8_t getLastI2CError();
    int16_t getLastErrorCode();

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    void printModuleInfo();
#endif

private:
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    TwoWire *_i2cWire;          // Wire class instance to use
#endif
    uint8_t _spiCSPin;          // SPI chip select pin
    SPISettings _spiSettings;   // SPI port settings to use
    LeptonFLiR_ImageStorageMode _storageMode; // Image data storage mode
    uint8_t *_imageData;        // Image data (column major)
    uint8_t *_spiFrameData;     // SPI frame data
    uint8_t *_telemetryData;    // SPI telemetry frame data
    uint8_t _lastI2CError;      // Last i2c error
    int16_t _lastErrorCode;     // Last error code

    int getImageDivFactor();
    uint8_t *getSPIFrameDataRow(int row);

    bool isBusy();
    bool waitBusy(int timeout = 0);

    void receiveStatus(bool *busy = NULL, bool *bootMode = NULL, bool *bootStatus = NULL);

    uint16_t commandCode(uint16_t cmdID, uint16_t cmdType);
    
    void sendCommand(uint16_t cmdCode);
    void sendCommand(uint16_t cmdCode, uint16_t value);
    void sendCommand(uint16_t cmdCode, uint32_t value);
    void sendCommand(uint16_t cmdCode, uint16_t *dataBuffer, int dataLength);

    int receiveCommand(uint16_t cmdCode, uint16_t *value);
    int receiveCommand(uint16_t cmdCode, uint32_t *value);
    int receiveCommand(uint16_t cmdCode, uint16_t *respBuffer, int maxLength);

    int sendReceiveCommand(uint16_t cmdCode, uint16_t *dataBuffer, int dataLength, uint16_t *respBuffer, int maxLength);

    int writeRegister(uint16_t regAddress, uint16_t *dataBuffer, int dataLength);
    int writeRegister(uint16_t regAddress, uint16_t *dataBuffer1, int dataLength1, uint16_t *dataBuffer2, int dataLength2);
    int readRegister(uint16_t regAddress, uint16_t *respBuffer, int respLength, int maxLength);
    int readRegister(uint16_t *respBuffer, int respLength, int maxLength);

#ifdef LEPFLIR_USE_SOFTWARE_I2C
    uint8_t _readBytes;
#endif
    void i2cWire_beginTransmission(uint8_t);
    uint8_t i2cWire_endTransmission(void);
    uint8_t i2cWire_requestFrom(uint8_t, uint8_t);
    size_t i2cWire_write(uint8_t);
    int i2cWire_read(void);
};

#endif
