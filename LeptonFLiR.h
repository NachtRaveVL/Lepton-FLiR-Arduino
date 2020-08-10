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

#ifndef LeptonFLiR_H
#define LeptonFLiR_H

// Library Setup

// NOTE: It is recommended to avoid editing library files directly and instead copy these
// into your own project and uncomment/define, as desired, before the include directive
// to this library, or through custom build flags.

// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
//#define LEPFLIR_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define to disable usage of the Scheduler library on SAM/SAMD architecures.
//#define LEPFLIR_DISABLE_SCHEDULER       1   // https://github.com/arduino-libraries/Scheduler

// Uncomment this define to disable 16 byte aligned memory allocations (may hinder performance).
//#define LEPFLIR_DISABLE_ALIGNED_MALLOC  1

// Uncomment this define if wanting to exclude extended i2c functions from compilation.
//#define LEPFLIR_EXCLUDE_EXT_I2C_FUNCS   1

// Uncomment this define to enable debug output.
//#define LEPFLIR_ENABLE_DEBUG_OUTPUT     1

// Hookup Instructions
// -PLEASE READ-
// Make sure to hookup the module's SPI lines MISO, MOSI, CLK (aka SCK), and CS (aka SS)
// correctly (Due, Zero, ATmega, etc. often use pins 50=MISO, 51=MOSI, 52=SCK, 53=SS, but
// one can just simply use the ICSP header pins ICSP-1=MISO, ICSP-4=MOSI, ICSP-3=SCK, which
// are consistent across all Arduino-specific boards). The module's MOSI line can simply
// be grounded since the module only uses SPI for slave-out data transfers (slave-in data
// transfers being ignored). The SS pin may be any digital output pin, with usage being
// active-low. The recommended VCC power supply and logic level is 3.3v, with 5v being
// supported *only* if your board can handle such (which often the case is not given the
// kinds of boards supported by this library). The two issolated power pins on the side of
// the FLiR module's breakout can safely be left disconnected. While the minimum SPI transfer
// rate is ~2.2MHz, the desired SPI transfer rate of 20MHz is used. The actual SPI transfer
// rate selected will be the first rate equal to or below 20MHz given the SPI clock divider
// (i.e. processor speed /2, /4, /8, /16, ..., /128).

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
#include "LeptonFLiRDefs.h"

#ifndef ENABLED
#define ENABLED  0x1
#endif
#ifndef DISABLED
#define DISABLED 0x0
#endif

typedef enum {
    TelemetryData_FFCState_NeverCommanded,
    TelemetryData_FFCState_InProgress,
    TelemetryData_FFCState_Complete
} TelemetryData_FFCState;

typedef struct {
    byte revisionMajor;
    byte revisionMinor;
    uint32_t cameraUptime;          // (milliseconds)
    bool ffcDesired;
    TelemetryData_FFCState ffcState;
    bool agcEnabled;                // def:disabled
    bool shutdownImminent;
    char serialNumber[24];
    char softwareRevision[12];
    uint32_t frameCounter;          // increments every 3rd frame, useful for determining new unique frame
    uint16_t frameMean;
    float fpaTemperature;           // min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    float housingTemperature;       // min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    uint32_t lastFFCTime;           // (milliseconds)
    float fpaTempAtLastFFC;         // min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    float housingTempAtLastFFC;     // min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    LEP_AGC_HISTOGRAM_ROI agcRegion;// min:0,0/end>beg, max:79,59/beg<end def:{0,0,79,59} (pixels)
    uint16_t agcClipHigh;           // min:0 max:4800 def:4800 (pixels)
    uint16_t agcClipLow;            // min:0 max:1024 def:512 (pixels)
    uint16_t log2FFCFrames;
} TelemetryData;

// Memory Footprint Note
// Image storage mode affects the total memory footprint. Memory constrained boards
// should take notice to the storage requirements. Note that the Lepton FLiR delivers
// 14bpp thermal image data with AGC mode disabled and 8bpp thermal image data with AGC
// mode enabled, therefore if using AGC mode always enabled it is more memory efficient
// to use an 8bpp mode to begin with. Note that with telemetry enabled, memory cost
// incurs an additional 164 bytes for telemetry data storage.
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

typedef enum {
    LeptonFLiR_TemperatureMode_Celsius,
    LeptonFLiR_TemperatureMode_Fahrenheit,
    LeptonFLiR_TemperatureMode_Kelvin,

    LeptonFLiR_TemperatureMode_Count
} LeptonFLiR_TemperatureMode;

class LeptonFLiR {
public:
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    // May use a different Wire instance than Wire. Some chipsets, such as Due/Zero/etc.,
    // have a Wire1 class instance that uses the SDA1/SCL1 lines instead.
    // Supported i2c baud rates are 100kHz, 400kHz, and 1000kHz.
    // Supported SPI baud rates are 2.2MHz to 20MHz.
    LeptonFLiR(TwoWire& i2cWire = Wire, byte spiCSPin = 53);
#else
    // Minimum supported i2c baud rate is 100kHz, which means minimum supported processor
    // speed is 4MHz+ while running i2c standard mode. For 400kHz i2c baud rate, minimum
    // supported processor speed is 16MHz+ while running i2c fast mode.
    // Supported SPI baud rates are 2.2MHz to 20MHz.
    LeptonFLiR(byte spiCSPin = 53);
#endif
    ~LeptonFLiR();

    // Called in setup()
    void init(LeptonFLiR_ImageStorageMode storageMode = LeptonFLiR_ImageStorageMode_80x60_16bpp, LeptonFLiR_TemperatureMode tempMode = LeptonFLiR_TemperatureMode_Celsius);

    byte getChipSelectPin();
    LeptonFLiR_ImageStorageMode getImageStorageMode();
    LeptonFLiR_TemperatureMode getTemperatureMode();

    // Image descriptors
    int getImageWidth();
    int getImageHeight();
    int getImageBpp(); // Bytes per pixel
    int getImagePitch(); // Bytes per row (may be different than Bpp * Width, in memory aligned mode)
    int getImageTotalBytes();

    // Image data access (disabled during frame read)
    byte *getImageData();
    byte *getImageDataRow(int row);
    uint16_t getImageDataRowCol(int row, int col);

    // Telemetry data access (disabled during frame read)
    byte *getTelemetryData(); // raw
    void getTelemetryData(TelemetryData *telemetry);

    // Commonly used properties from telemetry data
    uint32_t getTelemetryFrameCounter();
    bool getShouldRunFFCNormalization();

    // Sets fast enable/disable methods to call when enabling and disabling the SPI chip
    // select pin (e.g. PORTB |= 0x01, PORTB &= ~0x01, etc.). The function itself depends
    // on the board and pin used (see also digitalWriteFast library). Enable should set
    // the pin LOW, and disable should set the pin HIGH (aka active-low).
    typedef void(*digitalWriteFunc)(byte); // Passes pin number in
    void setFastCSFuncs(digitalWriteFunc csEnableFunc, digitalWriteFunc csDisableFunc);

    // This method reads the next image frame, taking up considerable processor time.
    // Returns a boolean indicating if next frame was successfully retrieved or not.
    bool readNextFrame();

    // AGC module commands

    void agc_setAGCEnabled(bool enabled); // def:disabled
    bool agc_getAGCEnabled();

    void agc_setAGCPolicy(LEP_AGC_POLICY policy); // def:LEP_AGC_HEQ
    LEP_AGC_POLICY agc_getAGCPolicy();

    void agc_setHEQScaleFactor(LEP_AGC_HEQ_SCALE_FACTOR factor); // def:LEP_AGC_SCALE_TO_8_BITS
    LEP_AGC_HEQ_SCALE_FACTOR agc_getHEQScaleFactor();

    void agc_setAGCCalcEnabled(bool enabled); // def:disabled
    bool agc_getAGCCalcEnabled();

    // SYS module commands

    void sys_getCameraStatus(LEP_SYS_CAM_STATUS *status);
    LEP_SYS_CAM_STATUS_STATES sys_getCameraStatus();

    void sys_getFlirSerialNumber(char *buffer, int maxLength = 16); // maxLength must at least be 16, recommended 20
    void sys_getCustomerSerialNumber(char *buffer, int maxLength = 64); // maxLength must at least be 64, recommended 80

    uint32_t sys_getCameraUptime(); // (milliseconds)

    float sys_getAuxTemperature(); // min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    float sys_getFPATemperature(); // min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)

    void sys_setTelemetryEnabled(bool enabled); // def:enabled
    bool sys_getTelemetryEnabled();

    void sys_runFFCNormalization();

    // VID module commands

    void vid_setPolarity(LEP_VID_POLARITY polarity); // def:LEP_VID_WHITE_HOT
    LEP_VID_POLARITY vid_getPolarity();

    void vid_setPseudoColorLUT(LEP_VID_PCOLOR_LUT table); // def:LEP_VID_FUSION_LUT
    LEP_VID_PCOLOR_LUT vid_getPseudoColorLUT(); 

    void vid_setFocusCalcEnabled(bool enabled); // def:disabled
    bool vid_getFocusCalcEnabled();

    void vid_setFreezeEnabled(bool enabled); // def:disabled
    bool vid_getFreezeEnabled();

#ifndef LEPFLIR_EXCLUDE_EXT_I2C_FUNCS

    // AGC extended module commands

    void agc_setHistogramRegion(LEP_AGC_HISTOGRAM_ROI *region); // min:0,0/end>beg, max:79,59/beg<end def:{0,0,79,59} (pixels)
    void agc_getHistogramRegion(LEP_AGC_HISTOGRAM_ROI *region);

    void agc_getHistogramStatistics(LEP_AGC_HISTOGRAM_STATISTICS *statistics); // min:{0,0,0,0} max:{0x3FFF,0x3FFF,0x3FFF,4800} (pixels)

    void agc_setHistogramClipPercent(uint16_t percent); // def:0
    uint16_t agc_getHistogramClipPercent();

    void agc_setHistogramTailSize(uint16_t size); // def:0
    uint16_t agc_getHistogramTailSize();

    void agc_setLinearMaxGain(uint16_t gain); // def:1
    uint16_t agc_getLinearMaxGain();

    void agc_setLinearMidpoint(uint16_t midpoint); // min:0 max:256 def:128
    uint16_t agc_getLinearMidpoint();

    void agc_setLinearDampeningFactor(uint16_t factor); // def:1
    uint16_t agc_getLinearDampeningFactor();

    void agc_setHEQDampeningFactor(uint16_t factor); // min:0 max:256 def:64
    uint16_t agc_getHEQDampeningFactor();

    void agc_setHEQMaxGain(uint16_t gain); // def:1
    uint16_t agc_getHEQMaxGain();

    void agc_setHEQClipLimitHigh(uint16_t limit); // min:0 max:4800 def:4800 (pixels)
    uint16_t agc_getHEQClipLimitHigh();

    void agc_setHEQClipLimitLow(uint16_t limit); // min:0 max:1024 def:512 (pixels)
    uint16_t agc_getHEQClipLimitLow();

    void agc_setHEQBinExtension(uint16_t extension); // def:0
    uint16_t agc_getHEQBinExtension();

    void agc_setHEQMidpoint(uint16_t midpoint); // min:0 max:256 def:128
    uint16_t agc_getHEQMidpoint();

    void agc_setHEQEmptyCounts(uint16_t counts); // min:0 max:0x3FFF def:2
    uint16_t agc_getHEQEmptyCounts();

    void agc_setHEQNormalizationFactor(uint16_t factor); // def:1
    uint16_t agc_getHEQNormalizationFactor();

    // SYS extended module commands

    void sys_runPingCamera(); // return put into lastLepResult

    void sys_setTelemetryLocation(LEP_SYS_TELEMETRY_LOCATION location); // def:LEP_TELEMETRY_LOCATION_FOOTER
    LEP_SYS_TELEMETRY_LOCATION sys_getTelemetryLocation();

    void sys_runFrameAveraging();

    void sys_setNumFramesToAverage(LEP_SYS_FRAME_AVERAGE average); // def:LEP_SYS_FA_DIV_8
    LEP_SYS_FRAME_AVERAGE sys_getNumFramesToAverage();

    void sys_getSceneStatistics(LEP_SYS_SCENE_STATISTICS *statistics);

    void sys_setSceneRegion(LEP_SYS_SCENE_ROI *region); // min:0,0/end>beg, max:79,59/beg<end def:{0,0,79,59} (pixels)
    void sys_getSceneRegion(LEP_SYS_SCENE_ROI *region);

    uint16_t sys_getThermalShutdownCount(); // min:0 max:65535 default:270 (pixels)

    void sys_setShutterPosition(LEP_SYS_SHUTTER_POSITION position); // def:LEP_SYS_SHUTTER_POSITION_UNKNOWN
    LEP_SYS_SHUTTER_POSITION sys_getShutterPosition();

    void sys_setFFCShutterMode(LEP_SYS_FFC_SHUTTER_MODE *mode); // see LEP_SYS_FFC_SHUTTER_MODE for defs
    void sys_getFFCShutterMode(LEP_SYS_FFC_SHUTTER_MODE *mode);

    LEP_SYS_FFC_STATUS sys_getFFCNormalizationStatus(); // def:LEP_SYS_FFC_STATUS_READY

    // VID extended module commands

    void vid_setUserColorLUT(LEP_VID_LUT_BUFFER *table); // These two methods may not work as intended, possibly leaving the I2C bus on the
    void vid_getUserColorLUT(LEP_VID_LUT_BUFFER *table); // FLiR in a non-responding state. A full power cycle may be needed to reset.

    void vid_setFocusRegion(LEP_VID_FOCUS_ROI *region); // min:1,1/end>beg+1, max:78,58/beg<end-1 def:{1,1,78,58} (pixels)
    void vid_getFocusRegion(LEP_VID_FOCUS_ROI *region);

    void vid_setFocusThreshold(uint32_t threshold); // def:30
    uint32_t vid_getFocusThreshold();

    uint32_t vid_getFocusMetric();

    void vid_setSceneBasedNUCEnabled(bool enabled); // def:enabled
    bool vid_getSceneBasedNUCEnabled();

    void vid_setGamma(uint32_t gamma); // def:58
    uint32_t vid_getGamma();

#endif

    // Module represents temperatures as kelvin x 100 (in integer format). These methods
    // convert to and from the selected temperature mode.
    float kelvin100ToTemperature(uint16_t kelvin100);
    uint16_t temperatureToKelvin100(float temperature);
    const char *getTemperatureSymbol();

    byte getLastI2CError();
    LEP_RESULT getLastLepResult();

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    void printModuleInfo();
    void checkForErrors();
#endif

private:
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    TwoWire *_i2cWire;          // Wire class instance to use
#endif
    byte _spiCSPin;             // SPI chip select pin
    SPISettings _spiSettings;   // SPI port settings to use
    LeptonFLiR_ImageStorageMode _storageMode; // Image data storage mode
    LeptonFLiR_TemperatureMode _tempMode; // Temperature display mode
    digitalWriteFunc _csEnableFunc; // Chip select enable function
    digitalWriteFunc _csDisableFunc; // Chip select disable function
    byte *_imageData;           // Image data (column major)
    byte *_spiFrameData;        // SPI frame data
    byte *_telemetryData;       // SPI telemetry frame data
    bool _isReadingNextFrame;   // Tracks if next frame is being read
    byte _lastI2CError;         // Last i2c error
    byte _lastLepResult;        // Last lep result

    byte *_getImageDataRow(int row);

    int getSPIFrameLines();
    int getSPIFrameTotalBytes();
    uint16_t *getSPIFrameDataRow(int row);

    bool waitCommandBegin(int timeout = 0);
    bool waitCommandFinish(int timeout = 0);

    uint16_t cmdCode(uint16_t cmdID, uint16_t cmdType);

    void sendCommand(uint16_t cmdCode);
    void sendCommand(uint16_t cmdCode, uint16_t value);
    void sendCommand(uint16_t cmdCode, uint32_t value);
    void sendCommand(uint16_t cmdCode, uint16_t *dataWords, int dataLength);

    void receiveCommand(uint16_t cmdCode, uint16_t *value);
    void receiveCommand(uint16_t cmdCode, uint32_t *value);
    void receiveCommand(uint16_t cmdCode, uint16_t *readWords, int maxLength);

    int writeCmdRegister(uint16_t cmdCode, uint16_t *dataWords, int dataLength);
    int readDataRegister(uint16_t *readWords, int maxLength);

    int writeRegister(uint16_t regAddress, uint16_t value);
    int readRegister(uint16_t regAddress, uint16_t *value);

#ifdef LEPFLIR_USE_SOFTWARE_I2C
    uint8_t _readBytes;
#endif
    void i2cWire_beginTransmission(uint8_t);
    uint8_t i2cWire_endTransmission(void);
    uint8_t i2cWire_requestFrom(uint8_t, uint8_t);
    size_t i2cWire_write(uint8_t);
    size_t i2cWire_write16(uint16_t);
    uint8_t i2cWire_read(void);
    uint16_t i2cWire_read16(void);
};

extern void wordsToHexString(uint16_t *dataWords, int dataLength, char *buffer, int maxLength);

extern float kelvin100ToCelsius(uint16_t kelvin100);
extern float kelvin100ToFahrenheit(uint16_t kelvin100);
extern float kelvin100ToKelvin(uint16_t kelvin100);
extern uint16_t celsiusToKelvin100(float celsius);
extern uint16_t fahrenheitToKelvin100(float fahrenheit);
extern uint16_t kelvinToKelvin100(float kelvin);

#endif
