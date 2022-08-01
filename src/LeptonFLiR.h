/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>

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

    Lepton-FLiR-Arduino - Version 0.9.92
*/

#ifndef LeptonFLiR_H
#define LeptonFLiR_H

// Library Setup

// NOTE: It is recommended to use custom build flags instead of editing this file directly.

// Uncomment or -D this define to enable usage of the software i2c library (min 4MHz+ processor).
//#define LEPFLIR_ENABLE_SOFTWARE_I2C             // https://github.com/felias-fogg/SoftI2CMaster

// Uncomment or -D this define to enable debug output (treats Serial output as attached to serial monitor).
//#define LEPFLIR_ENABLE_DEBUG_OUTPUT


#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#elif defined(__MBED__)
#include <mbed.h>
#else
#include <WProgram.h>
#endif
#include <assert.h>
#include <SPI.h>

#if defined(NDEBUG) && defined(LEPFLIR_ENABLE_DEBUG_OUTPUT)
#undef LEPFLIR_ENABLE_DEBUG_OUTPUT
#endif
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
#define LEPFLIR_SOFT_ASSERT(cond,msg)   LEPFLIR_softAssert((bool)(cond), String((msg)), __FILE__, __func__, __LINE__)
#define LEPFLIR_HARD_ASSERT(cond,msg)   LEPFLIR_hardAssert((bool)(cond), String((msg)), __FILE__, __func__, __LINE__)
extern void LEPFLIR_softAssert(bool, String, const char *, const char *, int);
extern void LEPFLIR_hardAssert(bool, String, const char *, const char *, int);
#else
#define LEPFLIR_SOFT_ASSERT(cond,msg)   ((void)0)
#define LEPFLIR_HARD_ASSERT(cond,msg)   ((void)0)
#endif

#ifndef LEPFLIR_ENABLE_SOFTWARE_I2C
#include <Wire.h>
#define LEPFLIR_USE_HARDWARE_I2C
#if BUFFER_LENGTH
#define LEPFLIR_I2C_BUFFER_LENGTH   BUFFER_LENGTH
#elif I2C_BUFFER_LENGTH
#define LEPFLIR_I2C_BUFFER_LENGTH   I2C_BUFFER_LENGTH
#else
#warning "i2c buffer length not defined - using default value of 32, which may not be correct for your microcontroller. Check Wire.h (or similar) for your hardware and manually define BUFFER_LENGTH or I2C_BUFFER_LENGTH to remove this warning."
#define LEPFLIR_I2C_BUFFER_LENGTH   32
#endif // /if BUFFER_LENGTH
#else
#define USE_SOFT_I2C_MASTER_H_AS_PLAIN_INCLUDE
#include "SoftI2CMaster.h"
#undef USE_SOFT_I2C_MASTER_H_AS_PLAIN_INCLUDE
#define LEPFLIR_USE_SOFTWARE_I2C
#endif // /ifndef LEPFLIR_ENABLE_SOFTWARE_I2C

// TODO: Include for TURBO SPI library

class LeptonFLiR;
#include "LeptonFLiRDefines.h"
#include "LeptonFLiRInlines.hpp"

class LeptonFLiR {
public:
#ifndef LEPFLIR_USE_SOFTWARE_I2C

    // Library constructor. Typically called during class instantiation, before setup().
    // ISR VSync pin only available for Lepton FLiR breakout board v2+ (GPIO3=VSYNC).
    // Boards with more than one i2c line (e.g. Due/Teensy/etc.) can supply a different
    // Wire instance, such as Wire1 (using SDA1/SCL1), Wire2 (using SDA2/SCL2), etc.
    // Supported i2c clock speeds are 100kHz, 400kHz, and 1000kHz.
    // Supported SPI clock speeds are ~2.2MHz(@80x60)/~8.8MHz(@160x120) to 20MHz.
    LeptonFLiR(byte spiCSPin = SS, byte isrVSyncPin = DISABLED, TwoWire& i2cWire = Wire, uint32_t i2cSpeed = 400000);

    // Convenience constructor for custom Wire instance. See main constructor.
    LeptonFLiR(TwoWire& i2cWire, uint32_t i2cSpeed = 400000, byte spiCSPin = SS, byte isrVSyncPin = DISABLED);

#else

    // Library constructor. Typically called during class instantiation, before setup().
    // ISR VSync pin only available for Lepton FLiR breakout board v2+ (GPIO3=VSYNC).
    // Minimum supported i2c clock speed is 100kHz, which sets minimum processor speed at
    // 4MHz+ running in i2c standard mode. For up to 400kHz i2c clock speeds, minimum
    // processor speed is 16MHz+ running in i2c fast mode.
    // Supported SPI clock speeds are ~2.2MHz(@80x60)/~8.8MHz(@160x120) to 20MHz.
    LeptonFLiR(byte spiCSPin = SS, byte isrVSyncPin = DISABLED);

#endif // /ifndef LEPFLIR_USE_SOFTWARE_I2C
    ~LeptonFLiR();

    // Initializes module. Typically called in setup().
    // See individual enums for more info.
    void init(LeptonFLiR_CameraType cameraType, LeptonFLiR_TemperatureMode tempMode = LeptonFLiR_TemperatureMode_Celsius);

    // Mode accessors
    byte getChipSelectPin();                                // CS pin
    byte getISRVSyncPin();                                  // ISR VSync pin
    uint32_t getI2CSpeed();                                 // i2c clock speed (Hz)
    LeptonFLiR_CameraType getCameraType();                  // Lepton camera type
    LeptonFLiR_TemperatureMode getTemperatureMode();        // Temperature mode

    // Image descriptors
    int getImageWidth();                                    // Image pixel width
    int getImageHeight();                                   // Image pixel height

    // This method attempts to read the next image frame, taking up considerable processor time.
    // Returns a boolean indicating if next frame was successfully retrieved or not.
    // This method will fail in the event of a desync upon frame read, mainly caused by
    // too slow of reading speed.
    bool tryReadNextFrame();


    // The following methods then operate on the most recently read frame, after tryReadNextFrame succeeds.

    // Frame settings
    uint32_t getFrameNumber();                              // Frame number, from last frame read
    LeptonFLiR_TelemetryMode getTelemetryMode();            // Telemetry enabled/location mode, from last frame read
    bool getAGCEnabled();                                   // AGC enabled mode, from last frame read
    bool getTLinearEnabled();                               // TLinear radiometry enabled mode, from last frame read
    bool getPseudoColorLUTEnabled();                        // Pseudo-color LUT (aka color palette) enabled mode, from last frame read

    // Raw image data buffer access
    bool isImageDataAvailable();                            // Raw image data availability flag, from last frame read (disabled during frame reads)
    LeptonFLiR_ImageMode getImageMode();                    // Raw image data mode, from last frame read
    int getImageBpp();                                      // Bytes per raw image pixel, from last frame read
    LeptonFLiR_PixelData getImagePixelData(int row, int col); // Individual pixel data accessor into raw image data, if available from last frame read

    // Processed image output data buffer access
    LeptonFLiR_ImageOutputMode getImageOutputMode();        // Processed image output data mode, based upon last frame read
    int getImageOutputBpp();                                // Bytes per processed image output pixel, based upon last frame read
    int getImageOutputPitch();                              // Bytes per processed image output row, based upon 16-byte alignment and last frame read
    int getImageOutputTotalSize();                          // Total byte size of processed image output, based upon 16-byte alignment and last frame read
    byte *getImageOutputData();                             // Creates a 16-byte aligned image data output data buffer (if not already created, owned by library code) processed from raw image data, if available from last frame read
    void getImageOutputData(byte* image, int pitch);        // Copies image output data into pre-allocated image data output buffer (owned by user code) processed from raw image data, if available from last frame read

    // Raw telemetry data buffer access
    bool isTelemetryDataAvailable();                        // Raw telemetry data availability flag, from last frame read (disabled during frame reads)
    uint32_t getTelemetryFrameCounter();                    // Frame counter from raw telemetry data, if available from last frame read
    bool getTelemetryShouldRunFFCNormalization();           // FFC normalization should run flag from raw telemetry data, if available from last frame read
    bool getTelemetryAGCEnabled();                          // AGC enabled flag from raw telemetry data, if available from last frame read

    // Processed telemetry output data buffer access
    LeptonFLiR_TelemetryData* getTelemetryOutputData();     // Creates a telemetry output data struct (if not already created, owned by library code) processed from raw telemetry data, if available from last frame read
    void getTelemetryOutputData(LeptonFLiR_TelemetryData *telemetry); // Copies telemetry output data into pre-allocated struct (owned by user code) processed from raw telemetry data, if available from last frame read


    // Modules of this class are separated into multiple files for organizational sake.
    // Each file is meant to expand upon the available methods of the LeptonFLiR class.
#define LEPFLIR_IN_PUBLIC
    #include "LeptonFLiR+AGC.h"                 // AGC module commands
    #include "LeptonFLiR+SYS.h"                 // SYS module commands
    #include "LeptonFLiR+VID.h"                 // VID module commands
    #include "LeptonFLiR+OEM.h"                 // OEM module commands
    #include "LeptonFLiR+RAD.h"                 // RAD module commands
    #include "LeptonFLiR+Utils.h"               // Utility methods
#undef LEPFLIR_IN_PUBLIC

protected:
#define LEPFLIR_IN_PROTECTED
    #include "LeptonFLiR+Protected.h"           // Protected members
    #include "LeptonFLiR+Wire.h"                // i2c communications
    #include "LeptonFLiR+SPI.h"                 // SPI communications
#undef LEPFLIR_IN_PROTECTED
};

#endif // /ifndef LeptonFLiR_H
