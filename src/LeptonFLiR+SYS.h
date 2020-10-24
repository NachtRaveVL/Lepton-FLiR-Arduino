/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / SYS Module Commands
*/
#ifndef LeptonFLiR_SYS_H
#define LeptonFLiR_SYS_H

#include "LeptonFLiR.h"

#ifndef LEPFLIR_IN_PUBLIC
#error "This file not meant to be directly included. Include LeptonFLiR.h instead."
#endif
//class LeptonFLiR {
//public:

    void sys_getCameraStatus(LEP_SYS_CAM_STATUS *status);
    LEP_SYS_CAM_STATUS_STATES sys_getCameraStatus();

    void sys_getFlirSerialNumber(char *buffer, int maxLength = 16); // maxLength must at least be 16, recommended 20
    void sys_getCustomerSerialNumber(char *buffer, int maxLength = 64); // maxLength must at least be 64, recommended 80

    uint32_t sys_getCameraUptime(); // (milliseconds)

    float sys_getAuxTemperature(); // min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)
    float sys_getFPATemperature(); // min:-273.15C max:382.20C (celsius), min:-459.67F max:719.96F (fahrenheit), min:0.00K max:655.35K (kelvin)

    void sys_setTelemetryEnabled(bool enabled); // def:enabled
    bool sys_getTelemetryEnabled();

    void sys_setTelemetryLocation(LEP_SYS_TELEMETRY_LOCATION location); // def:LEP_TELEMETRY_LOCATION_FOOTER
    LEP_SYS_TELEMETRY_LOCATION sys_getTelemetryLocation();

    void sys_runFFCNormalization();

    // TODO: Check to make sure we have all the SYS module commands up to v3.5. -NR

    void sys_runPingCamera(); // return put into lastLepResult

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

#endif // /ifndef LeptonFLiR_SYS_H
