/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>

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

// NOTE: It is recommended to avoid editing library files directly and instead use custom
// build flags. While most custom build systems support such, the Arduino IDE does not.
// Be aware that editing this file directly will affect all projects using this library.

// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
//#define LEPFLIR_ENABLE_SOFTWARE_I2C   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define if wanting to exclude extended i2c functions from compilation.
//#define LEPFLIR_EXCLUDE_EXT_I2C_FUNCS

// Uncomment this define to enable debug output.
#define LEPFLIR_ENABLE_DEBUG_OUTPUT

// Hookup Callout: SPI Data Line
// -PLEASE READ-
// Make sure to hookup the module's SPI lines MISO, MOSI, CLK (aka SCK), and CS (aka SS)
// correctly. Teensy 3.x uses pins 12=MISO, 11=MOSI, 13=SCK, and 10=SS, while ESP32(-S)
// uses pins 19=MISO, 23=MOSI, 16=SCK, and 5=SS. The module's MOSI line is optional and
// can simply be grounded since the module only uses SPI for slave-out data transfers
// (slave-in data transfers being ignored). The SS pin may be any digital output pin,
// with usage being active-low. The recommended VCC power supply and logic level is 3.3v.
// The two issolated power pins on the side of the FLiR v1.4 and v2 breakouts can safely
// be left disconnected. The minimum SPI transfer rate depends on the image resolution
// used by the sensor, with 80x60 displays requiring ~2.2MHz minimum, and 120x60 displays
// requiring ~X.XMHz minimum, while the maximum SPI transfer rate is 20MHz. The actual
// SPI transfer rate selected will be the first rate equal to or below 20MHz given the
// SPI clock divider (i.e. proc speed /2, /4, /8, ..., /128). Anything below 12MHz is
// considered sub-optimal, and may have difficulty maintaining VoSPI syncronization.

#if (defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)) && !defined(LEPFLIR_DISABLE_SCHEDULER)
#include "Scheduler.h"
#define LEPFLIR_USE_SCHEDULER
#endif

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#ifndef LEPFLIR_ENABLE_SOFTWARE_I2C
#include <Wire.h>
// Define BUFFER_LENGTH on platforms that don't natively define such.
#ifndef BUFFER_LENGTH
#ifdef I2C_BUFFER_LENGTH
#define BUFFER_LENGTH I2C_BUFFER_LENGTH
#else
#warning "i2c BUFFER_LENGTH not defined - using default of 32, which may not be supported by your microcontroller's hardware. Check Wire.h (or similar) file for your hardware and manually define to remove this warning."
#define BUFFER_LENGTH 32
#endif
#endif // /ifndef BUFFER_LENGTH
#endif // /ifndef LEPFLIR_ENABLE_SOFTWARE_I2C

#include <SPI.h>

#include "LeptonFLiRDefines.h"
#include "LeptonFLiRInlines.hpp"

class LeptonFLiR {
public:

#ifndef LEPFLIR_USE_SOFTWARE_I2C
    // May use a different Wire instance than Wire. Some chipsets, such as Due/Zero/etc.,
    // have a Wire1 class instance that uses the SDA1/SCL1 lines instead.
    // Supported i2c baud rates are 100kHz, 400kHz, and 1000kHz.
    // Supported SPI baud rates are 2.2MHz to 20MHz.
    // ISR VSync pin only available for Lepton FLiR breakout board v2+ (GPIO3=VSYNC).
    LeptonFLiR(TwoWire& i2cWire = Wire, byte spiCSPin = 10, byte isrVSyncPin = -1);
#else
    // Minimum supported i2c baud rate is 100kHz, which means minimum supported processor
    // speed is 4MHz+ while running i2c standard mode. For 400kHz i2c baud rate, minimum
    // supported processor speed is 16MHz+ while running i2c fast mode.
    // Supported SPI baud rates are 2.2MHz to 20MHz.
    // ISR VSync pin only available for Lepton FLiR breakout board v2+ (GPIO3=VSYNC).
    LeptonFLiR(byte spiCSPin = 10, byte isrVSyncPin = -1);
#endif
    ~LeptonFLiR();

    // Called in setup()
    void init(LeptonFLiR_CameraType cameraType, LeptonFLiR_TemperatureMode tempMode = LeptonFLiR_TemperatureMode_Celsius);

    byte getChipSelectPin();                                // CS pin
    byte getISRVSyncPin();                                  // ISR VSync pin
    LeptonFLiR_CameraType getCameraType();                  // Lepton camera type
    LeptonFLiR_TemperatureMode getTemperatureMode();        // Temperature mode

    // Sets fast enable/disable methods to call when enabling and disabling the SPI chip
    // select pin (e.g. PORTB |= 0x01, PORTB &= ~0x01, etc.). The function itself depends
    // on the board and pin used (see also digitalWriteFast library). Enable should set
    // the pin LOW, and disable should set the pin HIGH (aka active-low).
    typedef void(*digitalWriteFunc)(byte); // Passes pin number in
    void setFastCSFuncs(digitalWriteFunc csEnableFunc, digitalWriteFunc csDisableFunc);

    // Image descriptors
    int getImageWidth();                                    // Image pixel width
    int getImageHeight();                                   // Image pixel height

    // This method attempts to read the next image frame, taking up considerable processor time.
    // Returns a boolean indicating if next frame was successfully retrieved or not.
    // This method will fail in the event of a desync upon frame read.
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
    #include "LeptonFLiR+AGC.h"         // AGC module commands
    #include "LeptonFLiR+SYS.h"         // SYS module commands
    #include "LeptonFLiR+VID.h"         // VID module commands
    #include "LeptonFLiR+OEM.h"         // OEM module commands
    #include "LeptonFLiR+RAD.h"         // RAD module commands
    #include "LeptonFLiR+Utils.h"       // Utility methods
#undef LEPFLIR_IN_PUBLIC

protected:
#define LEPFLIR_IN_PROTECTED
    #include "LeptonFLiR+Protected.h"   // Protected members
    #include "LeptonFLiR+Wire.h"        // i2c communications
    #include "LeptonFLiR+SPI.h"         // SPI communications
#undef LEPFLIR_IN_PROTECTED
};

#endif // /#ifndef LeptonFLiR_H
