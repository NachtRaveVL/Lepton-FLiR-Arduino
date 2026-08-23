/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / OEM Module Commands
*/
#ifndef LeptonFLiR_OEM_H
#define LeptonFLiR_OEM_H

#include "LeptonFLiR.h"

#ifndef LEPFLIR_IN_PUBLIC
#error "This file not meant to be directly included. Include LeptonFLiR.h instead."
#endif
//class LeptonFLiR {
//public:

    // OEM module commands

    void oem_runPowerOn(); // After OEM power-down, host must first pulse SCL once to release SDA per FLIR wake sequence
    void oem_runPowerDown();

    void oem_getFlirPartNumber(char *buffer, int maxLength = 33);
    void oem_getSoftwareVersion(LEP_OEM_SW_VERSION *version);

    void oem_setVideoOutputEnabled(bool enabled);
    bool oem_getVideoOutputEnabled();

    void oem_setVideoOutputFormat(LEP_OEM_VIDEO_OUTPUT_FORMAT format);
    LEP_OEM_VIDEO_OUTPUT_FORMAT oem_getVideoOutputFormat();

    // Listed in Rev 303, but marked unsupported by that camera firmware release.
    void oem_setVideoOutputSource(LEP_OEM_VIDEO_OUTPUT_SOURCE source);
    LEP_OEM_VIDEO_OUTPUT_SOURCE oem_getVideoOutputSource();

    void oem_getCustomerPartNumber(char *buffer, int maxLength = 33);

    void oem_setVideoOutputSourceConstant(uint16_t value); // min:0 max:16383
    uint16_t oem_getVideoOutputSourceConstant();

    void oem_runReboot();

    void oem_setFFCNormalizationTarget(uint16_t target); // min:0 max:16383 def:8192
    uint16_t oem_getFFCNormalizationTarget();
    void oem_runFFCNormalization();
    LEP_OEM_STATUS oem_getStatus();

    uint16_t oem_getFrameMean();

    // Rev 303 lists the setter in the SDK API, but notes that only GET is supported by that firmware release.
    void oem_setGPIOMode(LEP_OEM_GPIO_MODE mode);
    LEP_OEM_GPIO_MODE oem_getGPIOMode();

    void oem_setVSyncDelay(LEP_OEM_VSYNC_DELAY delay);
    LEP_OEM_VSYNC_DELAY oem_getVSyncDelay();

    LEP_OEM_USER_PARAMS_STATE oem_getUserDefaultsState();
    void oem_runUserDefaultsCopyToOTP();
    void oem_runUserDefaultsRestore();

    void oem_setShutterProfile(LEP_OEM_SHUTTER_PROFILE *profile);
    void oem_getShutterProfile(LEP_OEM_SHUTTER_PROFILE *profile);

    void oem_setThermalShutdownEnabled(bool enabled);
    bool oem_getThermalShutdownEnabled();

    void oem_setBadPixelReplacementEnabled(bool enabled);
    bool oem_getBadPixelReplacementEnabled();

    void oem_setTemporalFilterEnabled(bool enabled);
    bool oem_getTemporalFilterEnabled();

    void oem_setColumnNoiseFilterEnabled(bool enabled);
    bool oem_getColumnNoiseFilterEnabled();

    void oem_setPixelNoiseFilterEnabled(bool enabled);
    bool oem_getPixelNoiseFilterEnabled();

#endif // /ifndef LeptonFLiR_OEM_H
