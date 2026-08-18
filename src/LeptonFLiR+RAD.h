/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / RAD Module Commands
*/
#ifndef LeptonFLiR_RAD_H
#define LeptonFLiR_RAD_H

#include "LeptonFLiR.h"

#ifndef LEPFLIR_IN_PUBLIC
#error "This file not meant to be directly included. Include LeptonFLiR.h instead."
#endif
//class LeptonFLiR {
//public:

    // RAD module commands

    void rad_setRBFOExternalParameters(LEP_RBFO *parameters);
    void rad_getRBFOExternalParameters(LEP_RBFO *parameters);

    void rad_setRadiometryEnabled(bool enabled);
    bool rad_getRadiometryEnabled();

    void rad_setTShutterMode(LEP_RAD_TS_MODE mode);
    LEP_RAD_TS_MODE rad_getTShutterMode();

    void rad_setTShutterTemperature(uint16_t kelvin100);
    uint16_t rad_getTShutterTemperature();

    void rad_runFFCNormalization();
    LEP_RAD_STATUS rad_getStatus();

    void rad_setFluxLinearParameters(LEP_RAD_FLUX_LINEAR_PARAMS *parameters);
    void rad_getFluxLinearParameters(LEP_RAD_FLUX_LINEAR_PARAMS *parameters);

    void rad_setTLinearEnabled(bool enabled);
    bool rad_getTLinearEnabled();

    void rad_setTLinearResolution(LEP_RAD_TLINEAR_RESOLUTION resolution);
    LEP_RAD_TLINEAR_RESOLUTION rad_getTLinearResolution();

    void rad_setTLinearAutoResolutionEnabled(bool enabled);
    bool rad_getTLinearAutoResolutionEnabled();

    void rad_setSpotmeterRegion(LEP_RAD_ROI *region);
    void rad_getSpotmeterRegion(LEP_RAD_ROI *region);
    void rad_getSpotmeterValues(LEP_RAD_SPOTMETER_VALUES *values);

    void rad_setLowGainRBFOExternalParameters(LEP_RBFO *parameters);
    void rad_getLowGainRBFOExternalParameters(LEP_RBFO *parameters);

#endif // /ifndef LeptonFLiR_RAD_H
