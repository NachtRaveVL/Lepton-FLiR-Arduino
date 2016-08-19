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

    Lepton-FLiR-Arduino - Version 0.9.5
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

// Uncomment this define if wanting to exclude extended i2c functions from compilation.
//#define LEPFLIR_EXCLUDE_EXT_I2C_FUNCS   1

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
#ifndef LEPFLIR_ENABLE_SOFTWARE_I2C
#include <Wire.h>
#endif
#include <SPI.h>
#include "LeptonFLiRDefs.h"

typedef enum {
    TelemetryData_FFCState_NeverCommanded,
    TelemetryData_FFCState_InProgress,
    TelemetryData_FFCState_Complete
} TelemetryData_FFCState;

typedef struct {
    uint8_t revisionMajor;
    uint8_t revisionMinor;
    uint32_t cameraUptime;
    bool ffcDesired;
    TelemetryData_FFCState ffcState;
    bool agcEnabled;
    bool shutdownImminent;
    char serialNumber[24];
    char softwareRevision[12];
    uint32_t frameCounter;
    uint16_t frameMean;
    float fpaTemperature;
    float housingTemperature;
    uint32_t lastFFCTime;
    float fpaTempAtLastFFC;
    float housingTempAtLastFFC;
    LEP_AGC_HISTOGRAM_ROI agcRegion;
    uint16_t agcClipHigh;
    uint16_t agcClipLow;
    uint16_t log2FFCFrames;
} TelemetryData;

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
    void init(LeptonFLiR_ImageStorageMode storageMode = LeptonFLiR_ImageStorageMode_80x60_16bpp, LeptonFLiR_TemperatureMode tempMode = LeptonFLiR_TemperatureMode_Celsius);

    uint8_t getChipSelectPin();
    LeptonFLiR_ImageStorageMode getImageStorageMode();
    LeptonFLiR_TemperatureMode getTemperatureMode();
    const char *getTemperatureSymbol();

    // Image descriptors
    int getImageWidth();
    int getImageHeight();
    int getImageBpp(); // Bytes per pixel
    int getImagePitch(); // Bytes per row (may be different than Bpp * Width, in memory aligned mode)
    int getImageTotalBytes();

    // Image data access (disabled during frame read)
    uint8_t *getImageData();
    uint8_t *getImageDataRow(int row);

    // Telemetry data access (disabled during frame read)
    uint8_t *getTelemetryData(); // raw
    void getTelemetryData(TelemetryData *telemetry);

    // This method reads the next image frame, taking up considerable processor time.
    // Returns a boolean indicating if next frame was successfully retrieved or not.
    bool readNextFrame();

    // AGC module commands

    void setAGCEnabled(bool enabled); // def:disabled
    bool getAGCEnabled();

    void setAGCPolicy(LEP_AGC_POLICY policy); // ???
    LEP_AGC_POLICY getAGCPolicy();

    void setAGCHEQScaleFactor(LEP_AGC_HEQ_SCALE_FACTOR factor); // def:LEP_AGC_SCALE_TO_8_BITS
    LEP_AGC_HEQ_SCALE_FACTOR getAGCHEQScaleFactor();

    void setAGCCalcEnabled(bool enabled); // def:disabled
    bool getAGCCalcEnabled();

    // SYS module commands

    void getSysCameraStatus(LEP_SYS_CAM_STATUS *status);
    LEP_SYS_CAM_STATUS_STATES getSysCameraStatus();

    void getSysFlirSerialNumber(char *buffer, int maxLength = 16); // maxLength must at least be 16, recommended 20
    void getSysCustomerSerialNumber(char *buffer, int maxLength = 64); // maxLength must at least be 64, recommended 80

    uint32_t getSysCameraUptime(); // (milliseconds)

    float getSysAuxTemperature(); // min:-273.15C/-459.67f max:382.20C/719.96f (celsius/fahrenheit)
    float getSysFPATemperature(); // min:-273.15C/-459.67f max:382.20C/719.96f (celsius/fahrenheit)

    void setSysTelemetryEnabled(bool enabled); // def:disabled
    bool getSysTelemetryEnabled();

    // VID module commands

    void setVidPolarity(LEP_VID_POLARITY polarity); // def:LEP_VID_WHITE_HOT
    LEP_VID_POLARITY getVidPolarity();

    void setVidPseudoColorLUT(LEP_VID_PCOLOR_LUT table); // def:LEP_VID_FUSION_LUT
    LEP_VID_PCOLOR_LUT getVidPseudoColorLUT(); 
    
    void setVidFocusCalcEnabled(bool enabled); // def:disabled
    bool getVidFocusCalcEnabled();

    void setVidFreezeEnabled(bool enabled); // def:disabled
    bool getVidFreezeEnabled();

#ifndef LEPFLIR_EXCLUDE_EXT_I2C_FUNCS

    // AGC extended module commands

    void setAGCHistogramRegion(LEP_AGC_HISTOGRAM_ROI *region); // min:0,0/end>beg, max:79,59/beg<end def:{0,0,79,59} (pixels)
    void getAGCHistogramRegion(LEP_AGC_HISTOGRAM_ROI *region);

    void getAGCHistogramStatistics(LEP_AGC_HISTOGRAM_STATISTICS *statistics); // min:{0,0,0,0} max:{0x3FFF,0x3FFF,0x3FFF,4800} (pixels)

    void setAGCHistogramClipPercent(uint16_t percent); // ???
    uint16_t getAGCHistogramClipPercent();

    void setAGCHistogramTailSize(uint16_t size); // ???
    uint16_t getAGCHistogramTailSize();

    void setAGCLinearMaxGain(uint16_t gain); // ???
    uint16_t getAGCLinearMaxGain();

    void setAGCLinearMidpoint(uint16_t midpoint); // ???
    uint16_t getAGCLinearMidpoint();

    void setAGCLinearDampeningFactor(uint16_t factor); // ???
    uint16_t getAGCLinearDampeningFactor();

    void setAGCHEQDampeningFactor(uint16_t factor); // min:0 max:256 def:64
    uint16_t getAGCHEQDampeningFactor();

    void setAGCHEQMaxGain(uint16_t gain); // ???
    uint16_t getAGCHEQMaxGain();

    void setAGCHEQClipLimitHigh(uint16_t limit); // min:0 max:4800 def:4800 (pixels)
    uint16_t getAGCHEQClipLimitHigh();

    void setAGCHEQClipLimitLow(uint16_t limit); // min:0 max:1024 def:512 (pixels)
    uint16_t getAGCHEQClipLimitLow();

    void setAGCHEQBinExtension(uint16_t extension); // ???
    uint16_t getAGCHEQBinExtension();

    void setAGCHEQMidpoint(uint16_t midpoint); // ???
    uint16_t getAGCHEQMidpoint();

    void setAGCHEQEmptyCounts(uint16_t counts); // min:0 max:0x3FFF def:2
    uint16_t getAGCHEQEmptyCounts();

    void setAGCHEQNormalizationFactor(uint16_t factor); // ???
    uint16_t getAGCHEQNormalizationFactor();

    // SYS extended module commands

    void runSysPingCamera(); // return put into lastLepResult

    void setSysTelemetryLocation(LEP_SYS_TELEMETRY_LOCATION location); // def:LEP_TELEMETRY_LOCATION_HEADER
    LEP_SYS_TELEMETRY_LOCATION getSysTelemetryLocation();

    void runSysFrameAveraging();

    void setSysNumFramesToAverage(LEP_SYS_FRAME_AVERAGE average); // def:LEP_SYS_FA_DIV_8
    LEP_SYS_FRAME_AVERAGE getSysNumFramesToAverage();

    void getSysSceneStatistics(LEP_SYS_SCENE_STATISTICS *statistics);

    void setSysSceneRegion(LEP_SYS_SCENE_ROI *region); // min:0,0/end>beg, max:79,59/beg<end def:{0,0,79,59} (pixels)
    void getSysSceneRegion(LEP_SYS_SCENE_ROI *region);

    uint16_t getSysThermalShutdownCount(); // min:0 max:65535 default:270 (pixels)

    void setSysShutterPosition(LEP_SYS_SHUTTER_POSITION position); // def:LEP_SYS_SHUTTER_POSITION_UNKNOWN
    LEP_SYS_SHUTTER_POSITION getSysShutterPosition();

    void setSysFFCShutterMode(LEP_SYS_FFC_SHUTTER_MODE *mode); // see LEP_SYS_FFC_SHUTTER_MODE for defs
    void getSysFFCShutterMode(LEP_SYS_FFC_SHUTTER_MODE *mode);

    void runSysFFCNormalization();

    LEP_SYS_FFC_STATUS getSysFFCNormalizationStatus(); // def:LEP_SYS_FFC_STATUS_READY

    // VID extended module commands

    void setVidUserColorLUT(LEP_VID_LUT_BUFFER *table);
    void getVidUserColorLUT(LEP_VID_LUT_BUFFER *table);

    void setVidFocusRegion(LEP_VID_FOCUS_ROI *region); // min:1,1/end>beg+1, max:78,58/beg<end-1 def:{1,1,78,58} (pixels)
    void getVidFocusRegion(LEP_VID_FOCUS_ROI *region);

    void setVidFocusThreshold(uint32_t threshold); // def:30
    uint32_t getVidFocusThreshold();

    uint32_t getVidFocusMetric();

    void setVidSceneBasedNUCEnabled(bool enabled); // ???
    bool getVidSceneBasedNUCEnabled();

    void setVidGamma(uint32_t gamma); // ???
    uint32_t getVidGamma();

#endif

    // Module represents temperatures as kelvin x 100 (in integer format). These methods
    // convert to and from the selected temperature mode.
    float kelvin100ToTemperature(uint16_t kelvin100);
    uint16_t temperatureToKelvin100(float temperature);
    
    uint8_t getLastI2CError();
    LEP_RESULT getLastLepResult();

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    void printModuleInfo();
    void checkForErrors();
#endif

private:
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    TwoWire *_i2cWire;          // Wire class instance to use
#endif
    uint8_t _spiCSPin;          // SPI chip select pin
    SPISettings _spiSettings;   // SPI port settings to use
    LeptonFLiR_ImageStorageMode _storageMode; // Image data storage mode
    LeptonFLiR_TemperatureMode _tempMode; // Temperature display mode
    uint8_t *_imageData;        // Image data (column major)
    uint8_t *_spiFrameData;     // SPI frame data
    uint8_t *_telemetryData;    // SPI telemetry frame data
    bool _isReadingNextFrame;   // Tracks if next frame is being read
    uint8_t _lastI2CError;      // Last i2c error
    uint8_t _lastLepResult;     // Last lep result

    uint8_t *_getImageDataRow(int row);

    int getSPIFrameLines();
    int getSPIFrameTotalBytes();
    uint8_t *getSPIFrameDataRow(int row);

    bool waitCommandBegin(int timeout = 0);
    bool waitCommandFinish(int timeout = 0);

    uint16_t commandCode(uint16_t cmdID, uint16_t cmdType);
    
    void sendCommand(uint16_t cmdCode);
    void sendCommand(uint16_t cmdCode, uint16_t value);
    void sendCommand(uint16_t cmdCode, uint32_t value);
    void sendCommand(uint16_t cmdCode, uint16_t *dataWords, int dataLength);

    int receiveCommand(uint16_t cmdCode, uint16_t *value);
    int receiveCommand(uint16_t cmdCode, uint32_t *value);
    int receiveCommand(uint16_t cmdCode, uint16_t *respBuffer, int maxLength);

    int sendReceiveCommand(uint16_t cmdCode, uint16_t *dataWords, int dataLength, uint16_t *respBuffer, int maxLength);

    int writeRegister(uint16_t regAddress, uint16_t *dataWords, int dataLength);
    int writeRegister(uint16_t regAddress, uint16_t *dataWords1, int dataLength1, uint16_t *dataWords2, int dataLength2);
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

extern void wordsToHexString(uint16_t *dataWords, int dataLength, char *buffer, int maxLength);

extern float kelvin100ToCelsius(uint16_t kelvin100);
extern float kelvin100ToFahrenheit(uint16_t kelvin100);
extern float kelvin100ToKelvin(uint16_t kelvin100);
extern uint16_t celsiusToKelvin100(float celsius);
extern uint16_t fahrenheitToKelvin100(float fahrenheit);
extern uint16_t kelvinToKelvin100(float kelvin);

#endif
