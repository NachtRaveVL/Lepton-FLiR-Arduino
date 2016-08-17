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

#ifndef LeptonFLiR_H
#define LeptonFLiR_H

// Library Setup

// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
//#define LEPFLIR_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define to disable usage of the Scheduler library on SAM/SAMD architecures.
//#define LEPFLIR_DISABLE_SCHEDULER       1   // https://github.com/arduino-libraries/Scheduler

// Uncomment this define to disable 16-byte aligned memory allocations (may hinder performance).
//#define LEPFLIR_DISABLE_ALIGNED_MALLOC  1

// Uncomment this define to enable debug output.
//#define LEPFLIR_ENABLE_DEBUG_OUTPUT     1

// Hookup Instructions
// Make sure to hookup the module's SPI lines MISO, MOSI, CLK (aka SCK), and CS (aka SS)
// correctly (Due, Zero, ATmega, etc. often use pins 50=MISO, 51=MOSI, 52=SCK, 53=SS, but
// one can just simply use the ICSP header pins ICSP-1=MISO, ICSP-4=MOSI, ICSP-3=SCK,
// which are consistent across all boards). The module's MOSI line can simply be grounded
// since the module only uses SPI for slave-out data transfers (slave-in data transfers
// being ignored). The SS pin may be any digital output pin, with usage being active-low.
// The recommended VCC power supply and logic level is 3.3v, but 5v also seems to work.
// The two issolated power pins on the side of the module's breakout can safely be left
// disconnected. The minimum SPI transfer rate is ~2.2MHz, which means one needs at least
// an 8MHz processor, but more realistically a 16MHz processor is likely required given
// the processing work involved to resize/BLIT the final image. The actual SPI transfer
// rate selected will be the first rate equal to or below 20MHz given the SPI clock
// divider (processor speed /2, /4, /8, /16, ..., /128).

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

// Memory Footprint Note
// Image storage mode affects the total memory footprint. Memory constrained boards
// should take notice to the storage requirements. Note that the Lepton FLiR delivers
// 14bpp thermal image data with AGC mode disabled and 8bpp thermal image data with AGC
// mode enabled, therefore if using AGC mode always enabled it is more memory efficient
// to use an 8bpp mode to begin with. Note that with telemetry enabled, memory cost
// incurs an additional 164 bytes for telemetry data storage. Lastly note that using
// the 80x60 16bpp mode is the most speed efficient since data transfers write directly
// to the image memory space without needing to perform software resizes/BLITs.
typedef enum {
    // Full 16bpp image mode, 9600 bytes for image data, 164 bytes for read frame (9604 bytes total, 9806 bytes if aligned)
    LeptonFLiR_ImageStorageMode_80x60_16bpp,
    // Full 8bpp image mode, 4800 bytes for image data, 164 bytes for read frame (4964 bytes total, 5006 bytes if aligned)
    LeptonFLiR_ImageStorageMode_80x60_8bpp,

    // Halved 16bpp image mode, 2400 bytes for image data, 328 bytes for read frame (2728 bytes total, 2782 bytes if aligned)
    LeptonFLiR_ImageStorageMode_40x30_16bpp,
    // Halved 8bpp image mode, 1200 bytes for image data, 328 bytes for read frame (1528 bytes total, 1814 bytes if aligned)
    LeptonFLiR_ImageStorageMode_40x30_8bpp,

    // Quartered 16bpp image mode, 600 bytes for image data, 656 bytes for read frame (1256 bytes total, 1446 bytes if aligned)
    LeptonFLiR_ImageStorageMode_20x15_16bpp,
    // Quartered 8bpp image mode, 300 bytes for image data, 656 bytes for read frame (956 bytes total, 1202 bytes if aligned)
    LeptonFLiR_ImageStorageMode_20x15_8bpp,

    LeptonFLiR_ImageStorageMode_Count
} LeptonFLiR_ImageStorageMode;

class LeptonFLiR {
public:
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    // May use a different Wire instance than Wire. Some chipsets, such as Due/Zero/etc.,
    // have a Wire1 class instance that uses the SDA1/SCL1 lines instead.
    // Supported i2c baud rates are 100kHz, 400kHz, and 1000kHz.
    // Supported SPI baud rates are 2.2MHz to 20MHz.
    LeptonFLiR(TwoWire& i2cWire = Wire, uint8_t spiCSPin = 53);
#else
    // Minimum supported i2c baud rate is 100kHz, which means minimum supported processor
    // speed is 4MHz+ while running i2c standard mode. For 400kHz i2c baud rate, minimum
    // supported processor speed is 16MHz+ while running i2c fast mode.
    // Supported SPI baud rates are 2.2MHz to 20MHz.
    LeptonFLiR(uint8_t spiCSPin = 53);
#endif
    ~LeptonFLiR();

    // Called in setup()
    void init(LeptonFLiR_ImageStorageMode storageMode = LeptonFLiR_ImageStorageMode_80x60_16bpp);

    // Image descriptors
    LeptonFLiR_ImageStorageMode getImageStorageMode();
    int getImageWidth();
    int getImageHeight();
    int getImageBpp(); // Bytes per pixel
    int getImagePitch(); // Bytes per row (may be different than Bpp * Width, in memory aligned mode)

    // Raw image data access (disabled during frame read)
    uint8_t *getImageData();
    uint8_t *getImageDataRow(int row);

    void setAGCEnabled(bool enabled);
    bool getAGCEnabled();

    void setTelemetryEnabled(bool enabled);
    bool getTelemetryEnabled();

    // Raw telemetry data access (disabled during frame read)
    uint8_t *getTelemetryData();

    // This method reads the next image frame, taking up considerable processor time.
    // Returns a boolean indicating if next frame was successfully retrieved or not.
    bool readNextFrame();

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
    bool _isReadingNextFrame;   // Tracks if next frame is being read
    uint8_t _lastI2CError;      // Last i2c error
    int16_t _lastErrorCode;     // Last error code

    int getSPIFrameLines();
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
