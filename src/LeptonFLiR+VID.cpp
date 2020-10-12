/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / VID Module Commands
*/

#include "LeptonFLiR.h"

void LeptonFLiR::vid_setPolarity(LEP_VID_POLARITY polarity) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setPolarity");
#endif

    sendCommand(cmdCode(LEP_CID_VID_POLARITY_SELECT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)polarity);
}

LEP_VID_POLARITY LeptonFLiR::vid_getPolarity() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getPolarity");
#endif

    uint32_t polarity;
    receiveCommand(cmdCode(LEP_CID_VID_POLARITY_SELECT, LEP_I2C_COMMAND_TYPE_GET), &polarity);
    return (LEP_VID_POLARITY)polarity;
}

void LeptonFLiR::vid_setPseudoColorLUT(LEP_VID_PCOLOR_LUT mode) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setPseudoColorLUT");
#endif

    sendCommand(cmdCode(LEP_CID_VID_LUT_SELECT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)mode);
}

LEP_VID_PCOLOR_LUT LeptonFLiR::vid_getPseudoColorLUT() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getPseudoColorLUT");
#endif

    uint32_t mode;
    receiveCommand(cmdCode(LEP_CID_VID_LUT_SELECT, LEP_I2C_COMMAND_TYPE_GET), &mode);
    return (LEP_VID_PCOLOR_LUT)mode;
}

void LeptonFLiR::vid_setFocusCalcEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setFocusCalcEnabled");
#endif

    sendCommand(cmdCode(LEP_CID_VID_FOCUS_CALC_ENABLE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::vid_getFocusCalcEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getFocusCalcEnabled");
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_CALC_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::vid_setFreezeEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setFreezeEnabled");
#endif

    sendCommand(cmdCode(LEP_CID_VID_FREEZE_ENABLE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::vid_getFreezeEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getFreezeEnabled");
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_VID_FREEZE_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::vid_setOutputFormat(LEP_VID_VIDEO_OUTPUT_FORMAT format) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setOutputFormat");
#endif

    sendCommand(cmdCode(LEP_CID_VID_OUTPUT_FORMAT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)format);
}

LEP_VID_VIDEO_OUTPUT_FORMAT LeptonFLiR::vid_getOutputFormat() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getOutputFormat");
#endif

    uint32_t format;
    receiveCommand(cmdCode(LEP_CID_VID_OUTPUT_FORMAT, LEP_I2C_COMMAND_TYPE_GET), &format);
    return (LEP_VID_VIDEO_OUTPUT_FORMAT)format;
}

void LeptonFLiR::vid_setUserColorLUT(LEP_VID_LUT_BUFFER *table) {
    if (!table) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setUserColorLUT");
#endif

    sendCommand(cmdCode(LEP_CID_VID_LUT_TRANSFER, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)table, sizeof(LEP_VID_LUT_BUFFER) / 2);
}

void LeptonFLiR::vid_getUserColorLUT(LEP_VID_LUT_BUFFER *table) {
    if (!table) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getUserColorLUT");
#endif

    receiveCommand(cmdCode(LEP_CID_VID_LUT_TRANSFER, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)table, sizeof(LEP_VID_LUT_BUFFER) / 2);
}

void LeptonFLiR::vid_setFocusRegion(LEP_VID_FOCUS_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setFocusRegion");
#endif

    sendCommand(cmdCode(LEP_CID_VID_FOCUS_ROI, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)region, sizeof(LEP_VID_FOCUS_ROI) / 2);
}

void LeptonFLiR::vid_getFocusRegion(LEP_VID_FOCUS_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getFocusRegion");
#endif

    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_ROI, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)region, sizeof(LEP_VID_FOCUS_ROI) / 2);
}

void LeptonFLiR::vid_setFocusThreshold(uint32_t threshold) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setFocusThreshold");
#endif

    sendCommand(cmdCode(LEP_CID_VID_FOCUS_THRESHOLD, LEP_I2C_COMMAND_TYPE_SET), threshold);
}

uint32_t LeptonFLiR::vid_getFocusThreshold() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getFocusThreshold");
#endif

    uint32_t threshold;
    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_THRESHOLD, LEP_I2C_COMMAND_TYPE_GET), &threshold);
    return threshold;
}

uint32_t LeptonFLiR::vid_getFocusMetric() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getFocusMetric");
#endif

    uint32_t metric;
    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_METRIC, LEP_I2C_COMMAND_TYPE_GET), &metric);
    return metric;
}

void LeptonFLiR::vid_setSceneBasedNUCEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setSceneBasedNUCEnabled");
#endif

    sendCommand(cmdCode(LEP_CID_VID_SBNUC_ENABLE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::vid_getSceneBasedNUCEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getSceneBasedNUCEnabled");
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_VID_SBNUC_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::vid_setGamma(uint32_t gamma) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_setGamma");
#endif

    sendCommand(cmdCode(LEP_CID_VID_GAMMA_SELECT, LEP_I2C_COMMAND_TYPE_SET), gamma);
}

uint32_t LeptonFLiR::vid_getGamma() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::vid_getGamma");
#endif

    uint32_t gamma;
    receiveCommand(cmdCode(LEP_CID_VID_GAMMA_SELECT, LEP_I2C_COMMAND_TYPE_GET), &gamma);
    return gamma;
}
