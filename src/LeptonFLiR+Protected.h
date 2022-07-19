/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / Protected Members
*/
#ifndef LeptonFLiR_Protected_H
#define LeptonFLiR_Protected_H

#include "LeptonFLiR.h"

#ifndef LEPFLIR_IN_PROTECTED
#error "This file not meant to be directly included. Include LeptonFLiR.h instead."
#endif
//class LeptonFLiR {
//protected:
    struct FrameSettings {
        FrameSettings(FrameSettings *frame, uint32_t frameNum);
        ~FrameSettings();

        uint32_t frameNumber;                               // Frame number (internal)
        LeptonFLiR_TelemetryMode telemetryMode;             // Telemetry enabled/location flag
        bool agcEnabled;                                    // AGC enabled flag
        bool tlinearEnabled;                                // TLinear radiometry enabled flag
        bool pclutEnabled;                                  // Pseudo-color LUT (aka color palette) enabled flag
        LeptonFLiR_ImageMode imageMode;                     // Image data storage mode
        LeptonFLiR_ImageOutputMode outputMode;              // Image data output mode (corresponding to storage mode)
        // Values valid only after frame prepare
        uint16_t *offsetTable;                              // Raw image data segment # (line/section) -> raw image data offset table (owned ptr)
        // Remaining values valid only after read
        const byte *imageData;                              // Raw image data in VoSPI frame buffer, after 2B ID (unowned ptr, pitch of SPI frame line size)
        const byte *telemetryData;                          // Raw telemetry data in VoSPI frame buffer, after 2B ID (unowned ptr, pitch of SPI frame line size)
    };

    byte _spiCSPin;                                         // SPI chip select pin (default: SS)
    byte _isrVSyncPin;                                      // ISR vsync pin (Lepton FLiR Breakout v2+ only) (default: DISABLED)
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    TwoWire *_i2cWire;                                      // Wire class instance (unowned) (default: Wire)
    int _i2cSpeed;                                          // Module's i2c clock speed (default: 400000)
#endif
    SPISettings _spiSettings;                               // SPI port settings
    LeptonFLiR_CameraType _cameraType;                      // Lepton camera type
    LeptonFLiR_TemperatureMode _tempMode;                   // Temperature display mode (default: Celsius)

    byte *_frameData;                                       // 16-byte-aligned SPI frame data (VoSPI packet) buffer (unowned)
    byte *_frameData_orig;                                  // Non-aligned SPI frame data (VoSPI packet) buffer (original alloc ptr, owned)
    int _frameDataSize_orig;                                // Frame data size in bytes (used for realloc)

    byte *_imageOutput;                                     // 16-byte-aligned image output data (if created, unowned)
    byte *_imageOutput_orig;                                // Non-aligned image output data (if created, original alloc ptr, owned)
    int _imageOutputSize_orig;                              // Image output data size in bytes (used for realloc)

    LeptonFLiR_TelemetryData *_telemetryOutput;             // Telemetry output (if created, owned)

    uint32_t _frameCounter;                                 // Frame counter (internal)
    FrameSettings *_lastFrame;                              // Last frame settings (if created, owned)
    FrameSettings *_nextFrame;                              // Next frame settings (if created, owned)
    bool _nextFrameNeedsUpdate;                             // Flag tracking if next frame settings needs refreshed

    bool _isReadingNextFrame;                               // Tracks if next frame is being read
    byte _lastI2CError;                                     // Last module i2c error
    byte _lastLepResult;                                    // Last module command result

    FrameSettings* getNextFrame();                          // Resolves for next frame, updating/advancing as needed
    void updateNextFrame();                                 // Updates next frame settings to resolved current
    void advanceNextFrame();                                // Advances next frame by pushing it to last frame, and/or creating anew
    void prepareNextFrame();                                // Prepares SPI buffer for next frame, allocating memory to such

    uint32_t getNextFrameNumber();                          // Frame number, from current settings
    LeptonFLiR_ImageMode getNextImageMode();                // Raw image data mode, from current settings
    LeptonFLiR_ImageOutputMode getNextImageOutputMode();    // Processed image output data mode, from current settings
    LeptonFLiR_TelemetryMode getNextTelemetryMode();        // Telemetry enabled/location mode, from current settings
    bool getNextAGCEnabled();                               // AGC enabled mode, from current settings
    bool getNextTLinearEnabled();                           // TLinear radiometry enabled mode, from current settings
    bool getNextPseudoColorLUTEnabled();                    // Pseudo-color LUT (aka color palette) enabled mode, from current settings

    int getSPIClockDivisor();                               // SPI clock divisor (approximation)
    int getSPIFrameLineSize();                              // SPI frame line (VoSPI packet) size, in bytes
    int getSPIFrameLineSize16();                            // SPI frame line (VoSPI packet) size, in words
    int getSPIFrameDataSize();                              // SPI frame line (VoSPI packet) data payload size, in bytes
    int getSPIFrameDataSize16();                            // SPI frame line (VoSPI packet) data payload size, in words
    int getSPIFrameTelemetryLines();                        // Number of raw telemetry data SPI frame lines (VoSPI packets)
    int getSPIFrameImageLines();                            // Number of raw image data SPI frame lines (VoSPI packets)
    int getSPIFrameTotalLines();                            // Total number of SPI frame lines (VoSPI packets)
    int getSPIFrameTotalSize();                             // Total size of SPI frame data (VoSPI packet) buffer, in bytes
    int getSPIFrameTotalSize16();                           // Total size of SPI frame data (VoSPI packet) buffer, in words
    uint16_t *getSPIFrameData(int line);                    // Raw SPI frame data (VoSPI packet) accessor (unowned ptr)

    const byte *getImageData(int row, int section);         // Raw image data accessor (unowned ptr)

    const byte *getTelemetryData(int row);                  // Raw telemetry data accessor (unowned ptr)

    static float kelvin100ToCelsius(uint16_t kelvin100);
    static float kelvin100ToFahrenheit(uint16_t kelvin100);
    static float kelvin100ToKelvin(uint16_t kelvin100);
    static uint16_t celsiusToKelvin100(float celsius);
    static uint16_t fahrenheitToKelvin100(float fahrenheit);
    static uint16_t kelvinToKelvin100(float kelvin);

#endif // /ifndef LeptonFLiR_Protected_H
