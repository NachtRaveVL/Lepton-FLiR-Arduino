/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / RAD Module Commands
*/

#include "LeptonFLiR.h"

namespace {

uint32_t radState(bool enabled) {
    return enabled ? 1u : 0u;
}

} // namespace

void LeptonFLiR::rad_setRBFOExternalParameters(LEP_RBFO *parameters) {
    if (!parameters) return;
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_setRBFOExternalParameters"));
#endif
    sendCommand(cmdCode(LEP_CID_RAD_RBFO_PARAMS, LEP_I2C_COMMAND_TYPE_SET),
                (uint16_t *)parameters, sizeof(LEP_RBFO) / 2);
}

void LeptonFLiR::rad_getRBFOExternalParameters(LEP_RBFO *parameters) {
    if (!parameters) return;
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getRBFOExternalParameters"));
#endif
    receiveCommand(cmdCode(LEP_CID_RAD_RBFO_PARAMS, LEP_I2C_COMMAND_TYPE_GET),
                   (uint16_t *)parameters, sizeof(LEP_RBFO) / 2);
}

void LeptonFLiR::rad_setRadiometryEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_setRadiometryEnabled"));
#endif
    sendCommand(cmdCode(LEP_CID_RAD_RADIOMETRY_ENABLE, LEP_I2C_COMMAND_TYPE_SET), radState(enabled));
}

bool LeptonFLiR::rad_getRadiometryEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getRadiometryEnabled"));
#endif
    uint32_t enabled = 0;
    receiveCommand(cmdCode(LEP_CID_RAD_RADIOMETRY_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled != 0;
}

void LeptonFLiR::rad_setTShutterMode(LEP_RAD_TS_MODE mode) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_setTShutterMode"));
#endif
    sendCommand(cmdCode(LEP_CID_RAD_TSHUTTER_MODE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)mode);
}

LEP_RAD_TS_MODE LeptonFLiR::rad_getTShutterMode() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getTShutterMode"));
#endif
    uint32_t mode = 0;
    receiveCommand(cmdCode(LEP_CID_RAD_TSHUTTER_MODE, LEP_I2C_COMMAND_TYPE_GET), &mode);
    return (LEP_RAD_TS_MODE)mode;
}

void LeptonFLiR::rad_setTShutterTemperature(uint16_t kelvin100) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_setTShutterTemperature"));
#endif
    sendCommand(cmdCode(LEP_CID_RAD_TSHUTTER_TEMP, LEP_I2C_COMMAND_TYPE_SET), kelvin100);
}

uint16_t LeptonFLiR::rad_getTShutterTemperature() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getTShutterTemperature"));
#endif
    uint16_t kelvin100 = 0;
    receiveCommand(cmdCode(LEP_CID_RAD_TSHUTTER_TEMP, LEP_I2C_COMMAND_TYPE_GET), &kelvin100);
    return kelvin100;
}

void LeptonFLiR::rad_runFFCNormalization() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_runFFCNormalization"));
#endif
    sendCommand(cmdCode(LEP_CID_RAD_FFC_NORMALIZATION, LEP_I2C_COMMAND_TYPE_RUN));
}

LEP_RAD_STATUS LeptonFLiR::rad_getStatus() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getStatus"));
#endif
    uint32_t status = 0;
    receiveCommand(cmdCode(LEP_CID_RAD_RUN_STATUS, LEP_I2C_COMMAND_TYPE_GET), &status);
    return (LEP_RAD_STATUS)status;
}

void LeptonFLiR::rad_setFluxLinearParameters(LEP_RAD_FLUX_LINEAR_PARAMS *parameters) {
    if (!parameters) return;
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_setFluxLinearParameters"));
#endif
    sendCommand(cmdCode(LEP_CID_RAD_FLUX_LINEAR_PARAMS, LEP_I2C_COMMAND_TYPE_SET),
                (uint16_t *)parameters, sizeof(LEP_RAD_FLUX_LINEAR_PARAMS) / 2);
}

void LeptonFLiR::rad_getFluxLinearParameters(LEP_RAD_FLUX_LINEAR_PARAMS *parameters) {
    if (!parameters) return;
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getFluxLinearParameters"));
#endif
    receiveCommand(cmdCode(LEP_CID_RAD_FLUX_LINEAR_PARAMS, LEP_I2C_COMMAND_TYPE_GET),
                   (uint16_t *)parameters, sizeof(LEP_RAD_FLUX_LINEAR_PARAMS) / 2);
}

void LeptonFLiR::rad_setTLinearEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_setTLinearEnabled"));
#endif
    sendCommand(cmdCode(LEP_CID_RAD_TLINEAR_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), radState(enabled));
    _nextFrameNeedsUpdate = true;
}

bool LeptonFLiR::rad_getTLinearEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getTLinearEnabled"));
#endif
    uint32_t enabled = 0;
    receiveCommand(cmdCode(LEP_CID_RAD_TLINEAR_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled != 0;
}

void LeptonFLiR::rad_setTLinearResolution(LEP_RAD_TLINEAR_RESOLUTION resolution) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_setTLinearResolution"));
#endif
    sendCommand(cmdCode(LEP_CID_RAD_TLINEAR_RESOLUTION, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)resolution);
}

LEP_RAD_TLINEAR_RESOLUTION LeptonFLiR::rad_getTLinearResolution() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getTLinearResolution"));
#endif
    uint32_t resolution = 0;
    receiveCommand(cmdCode(LEP_CID_RAD_TLINEAR_RESOLUTION, LEP_I2C_COMMAND_TYPE_GET), &resolution);
    return (LEP_RAD_TLINEAR_RESOLUTION)resolution;
}

void LeptonFLiR::rad_setTLinearAutoResolutionEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_setTLinearAutoResolutionEnabled"));
#endif
    sendCommand(cmdCode(LEP_CID_RAD_TLINEAR_AUTO_RESOLUTION, LEP_I2C_COMMAND_TYPE_SET), radState(enabled));
}

bool LeptonFLiR::rad_getTLinearAutoResolutionEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getTLinearAutoResolutionEnabled"));
#endif
    uint32_t enabled = 0;
    receiveCommand(cmdCode(LEP_CID_RAD_TLINEAR_AUTO_RESOLUTION, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled != 0;
}

void LeptonFLiR::rad_setSpotmeterRegion(LEP_RAD_ROI *region) {
    if (!region) return;
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_setSpotmeterRegion"));
#endif
    sendCommand(cmdCode(LEP_CID_RAD_SPOTMETER_ROI, LEP_I2C_COMMAND_TYPE_SET),
                (uint16_t *)region, sizeof(LEP_RAD_ROI) / 2);
}

void LeptonFLiR::rad_getSpotmeterRegion(LEP_RAD_ROI *region) {
    if (!region) return;
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getSpotmeterRegion"));
#endif
    receiveCommand(cmdCode(LEP_CID_RAD_SPOTMETER_ROI, LEP_I2C_COMMAND_TYPE_GET),
                   (uint16_t *)region, sizeof(LEP_RAD_ROI) / 2);
}

void LeptonFLiR::rad_getSpotmeterValues(LEP_RAD_SPOTMETER_VALUES *values) {
    if (!values) return;
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getSpotmeterValues"));
#endif
    receiveCommand(cmdCode(LEP_CID_RAD_SPOTMETER_VALUES, LEP_I2C_COMMAND_TYPE_GET),
                   (uint16_t *)values, sizeof(LEP_RAD_SPOTMETER_VALUES) / 2);
}

void LeptonFLiR::rad_setLowGainRBFOExternalParameters(LEP_RBFO *parameters) {
    if (!parameters) return;
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_setLowGainRBFOExternalParameters"));
#endif
    sendCommand(cmdCode(LEP_CID_RAD_LOW_GAIN_RBFO_PARAMS, LEP_I2C_COMMAND_TYPE_SET),
                (uint16_t *)parameters, sizeof(LEP_RBFO) / 2);
}

void LeptonFLiR::rad_getLowGainRBFOExternalParameters(LEP_RBFO *parameters) {
    if (!parameters) return;
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::rad_getLowGainRBFOExternalParameters"));
#endif
    receiveCommand(cmdCode(LEP_CID_RAD_LOW_GAIN_RBFO_PARAMS, LEP_I2C_COMMAND_TYPE_GET),
                   (uint16_t *)parameters, sizeof(LEP_RBFO) / 2);
}
