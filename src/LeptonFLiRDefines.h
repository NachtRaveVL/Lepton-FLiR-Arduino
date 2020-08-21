/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR Defines
*/

#ifndef LeptonFLiRDefines_H
#define LeptonFLiRDefines_H

#include "LeptonFLiRIDDDefs.h"

#ifndef FLT_EPSILON
#define FLT_EPSILON                     0.00001f            // Floating point error tolerance
#endif
#ifndef ENABLED
#define ENABLED                         0x1                 // Enabled define (convenience)
#endif
#ifndef DISABLED
#define DISABLED                        0x0                 // Disabled define (convenience)
#endif

#define LEPFLIR_GEN_CMD_TIMEOUT         5000                // Timeout for commands to be processed
#define LEPFLIR_SPI_MAX_SPEED           20000000            // Maximum SPI speed for FLiR module
#define LEPFLIR_SPI_OPTIMAL_MIN_SPEED   12000000            // Minimum optimal SPI speed for FLiR module
#define LEPFLIR_SPI_MIN_SPEED           2200000             // Minimum SPI speed for FLiR module

enum LeptonFLiR_CameraType {
    LeptonFLiR_CameraType_Lepton1,              // Lepton v1 camera, running at 80x60
    LeptonFLiR_CameraType_Lepton1_6,            // Lepton v1.6 camera, running at 80x60
    LeptonFLiR_CameraType_Lepton2,              // Lepton v2 camera, running at 80x60
    LeptonFLiR_CameraType_Lepton2_5,            // Lepton v2.5 camera, running at 80x60
    LeptonFLiR_CameraType_Lepton3,              // Lepton v3 camera, running at 160x120
    LeptonFLiR_CameraType_Lepton3_5,            // Lepton v3.5 camera, running at 160x120

    LeptonFLiR_CameraType_Count,                // Internal use only
    LeptonFLiR_CameraType_Undefined = -1        // Internal use only
};

enum LeptonFLiR_TemperatureMode {
    LeptonFLiR_TemperatureMode_Celsius,         // Celsius temperature mode
    LeptonFLiR_TemperatureMode_Fahrenheit,      // Fahrenheit temperature mode
    LeptonFLiR_TemperatureMode_Kelvin,          // Kelvin temperature mode

    LeptonFLiR_TemperatureMode_Count,           // Internal use only
    LeptonFLiR_TemperatureMode_Undefined = -1   // Internal use only
};

enum LeptonFLiR_ImageMode {
    // 80x60 24bpp RGB888 pseduo-color LUT (aka color palette) image mode (Lepton v1-v2.5), 244 byte read frame (no telemetry).
    LeptonFLiR_ImageMode_80x60_24bpp_244Brf,
    // 80x60 8/14/16bpp grayscale image mode (Lepton v1-v2.5), 164 byte read frame.
    LeptonFLiR_ImageMode_80x60_16bpp_164Brf,

    // 160x120 24bpp RGB888 pseduo-color LUT (aka color palette) image mode (Lepton v3+), 244 byte read frame (no telemetry).
    LeptonFLiR_ImageMode_160x120_24bpp_244Brf,
    // 160x120 8/14/16bpp grayscale image mode (Lepton v3+), 164 byte read frame.
    LeptonFLiR_ImageMode_160x120_16bpp_164Brf,

    LeptonFLiR_ImageMode_Count,                 // Internal use only
    LeptonFLiR_ImageMode_Undefined = -1         // Internal use only
};

enum LeptonFLiR_ImageOutputMode {
    LeptonFLiR_ImageOutputMode_GS8,             // 8bpp grayscale image output mode
    LeptonFLiR_ImageOutputMode_GS16,            // 16bpp grayscale image output mode
    LeptonFLiR_ImageOutputMode_RGB888,          // 24bpp RGB888 image output mode

    LeptonFLiR_ImageOutputMode_Count,           // Internal use only
    LeptonFLiR_ImageOutputMode_Undefined = -1   // Internal use only
};

enum LeptonFLiR_TelemetryMode {
    LeptonFLiR_TelemetryMode_Disabled,          // Telemetry disabled
    LeptonFLiR_TelemetryMode_Header,            // Telemetry enabled at header
    LeptonFLiR_TelemetryMode_Footer             // Telemetry enabled at footer
};

enum LeptonFLiR_TelemetryFFCState {
    LeptonFLiR_TelemetryFFCState_NeverCommanded,// FFC never commanded
    LeptonFLiR_TelemetryFFCState_InProgress,    // FFC in progress
    LeptonFLiR_TelemetryFFCState_Complete       // FFC completed
};

enum LeptonFLiR_TelemetryGainMode {
    LeptonFLiR_TelemetryGainMode_High,          // High gain mode
    LeptonFLiR_TelemetryGainMode_Low,           // Low gain mode
    LeptonFLiR_TelemetryGainMode_Auto           // Auto gain mode (not valid for effGainMode)
};

struct LeptonFLiR_TelemetryData {
    byte revisionMajor;                         // Telemetry major revision #
    byte revisionMinor;                         // Telemetry minor revision #
    uint32_t cameraUptime;                      // Camera uptime (milliseconds)
    bool ffcDesired;                            // FFC desired flag
    LeptonFLiR_TelemetryFFCState ffcState;      // FFC state
    bool agcEnabled;                            // AGC enabled flag
    bool shutdownImminent;                      // Shutdown imminent flag
    char serialNumber[24];                      // Camera serial number
    char softwareRevision[12];                  // Camera software revision
    uint32_t frameCounter;                      // Frame counter, increments every 3rd frame (export restriction), useful for determining new unique frame
    uint16_t frameMean;                         // Frame mean value
    float fpaTemperature;                       // Sensor temperature, min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    float housingTemperature;                   // Housting temperatur, min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    uint32_t lastFFCTime;                       // Last time FFC was ran (milliseconds)
    float fpaTempAtLastFFC;                     // Sensor temperature at last FFC, min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    float housingTempAtLastFFC;                 // Housing temperature at last FFC, min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    LEP_AGC_HISTOGRAM_ROI agcRegion;            // Region of interest for AGC histogram equalization, min:0,0 max:79,59 (Lepton v1-v2.5), min:0,0 max:159,119 (Lepton v3+), def:{0,0,max,max} (pixels)
    uint16_t agcClipHigh;                       // AGC clip upper bound, min:0 max:4800 def:4800 (pixels)
    uint16_t agcClipLow;                        // AGC clip lower bound min:0 max:1024 def:512 (pixels)
    uint16_t log2FFC;                           // Log2 of FFC
    LEP_VID_VIDEO_OUTPUT_FORMAT vidFormat;      // Video output format
    uint16_t sceneEmissivity;                   // Scene emissivity, min:0 max:8192 (8192=100%)
    uint16_t atmoTau;                           // Atmospheric transmission (tau), min:0 max:8192 (8192=100%)
    uint16_t windowTau;                         // Window transmission (tau), min:0 max:8192 (8192=100%)
    uint16_t windowReflTau;                     // Window reflection transmission (tau), min:0 max:8192 (8192=100%)
    float bgTemperature;                        // Background temperature, min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    float atmoTemperature;                      // Atmospheric temperature, min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    float windowTemperature;                    // Window temperature, min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    float windowReflTemperature;                // Window reflection temperature, min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    LeptonFLiR_TelemetryGainMode gainMode;      // Gain mode
    LeptonFLiR_TelemetryGainMode effGainMode;   // Effective gain mode (low/high resolved when gain mode = auto)
    bool gainModeSwitchDesired;                 // Gain mode switch desired flag
    float radGainModeSwitchHtLTemp;             // Gain mode threshold high-to-low temperature, when auto mode should switch from high to low mode (tlinear-disabled)
    float radGainModeSwitchLtHTemp;             // Gain mode threshold low-to-high temperature, when auto mode should switch from low to high mode (tlinear-disabled)
    float tlinearGainModeSwitchHtLTemp;         // Gain mode threshold high-to-low temperature, when auto mode should switch from high to low mode (tlinear-enabled)
};

union LeptonFLiR_PixelData {
    // 14bpp grayscale, standard mode
    struct {
        uint16_t value;
    } std;
    // 8bpp grayscale, AGC-enabled mode
    struct {
        uint8_t _res;
        uint8_t value;
    } agc;
    // 16bpp grayscale, TLinear radiometry-enabled mode
    struct {
        uint16_t value;
    } tlinear;
    // 24bpp RGB888 pseudo-color LUT (aka color palette)-enabled mode
    struct {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    } pclut;
};

#endif // /#ifndef LeptonFLiRDefines_H
