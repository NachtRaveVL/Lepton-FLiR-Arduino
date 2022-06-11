/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / AGC Module Commands
*/

#include "LeptonFLiR.h"

void LeptonFLiR::agc_setAGCEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setAGCEnabled"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::agc_getAGCEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getAGCEnabled"));
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::agc_setAGCPolicy(LEP_AGC_POLICY policy) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setAGCPolicy"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_POLICY, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)policy);
}

LEP_AGC_POLICY LeptonFLiR::agc_getAGCPolicy() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getAGCPolicy"));
#endif

    uint32_t policy;
    receiveCommand(cmdCode(LEP_CID_AGC_POLICY, LEP_I2C_COMMAND_TYPE_GET), &policy);
    return (LEP_AGC_POLICY)policy;
}

void LeptonFLiR::agc_setHEQScaleFactor(LEP_AGC_HEQ_SCALE_FACTOR factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHEQScaleFactor"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_SCALE_FACTOR, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)factor);
}

LEP_AGC_HEQ_SCALE_FACTOR LeptonFLiR::agc_getHEQScaleFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHEQScaleFactor"));
#endif

    uint32_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_SCALE_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return (LEP_AGC_HEQ_SCALE_FACTOR)factor;
}

void LeptonFLiR::agc_setAGCCalcEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setAGCCalcEnabled"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_CALC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::agc_getAGCCalcEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getAGCCalcEnabled"));
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_AGC_CALC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::agc_setHistogramRegion(LEP_AGC_HISTOGRAM_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHistogramRegion"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_ROI, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)region, sizeof(LEP_AGC_HISTOGRAM_ROI) / 2);
}

void LeptonFLiR::agc_getHistogramRegion(LEP_AGC_HISTOGRAM_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHistogramRegion"));
#endif

    receiveCommand(cmdCode(LEP_CID_AGC_ROI, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)region, sizeof(LEP_AGC_HISTOGRAM_ROI) / 2);
}

void LeptonFLiR::agc_getHistogramStatistics(LEP_AGC_HISTOGRAM_STATISTICS *statistics) {
    if (!statistics) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHistogramStatistics"));
#endif

    receiveCommand(cmdCode(LEP_CID_AGC_STATISTICS, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)statistics, sizeof(LEP_AGC_HISTOGRAM_STATISTICS) / 2);
}

void LeptonFLiR::agc_setHistogramClipPercent(uint16_t percent) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHistogramClipPercent"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_CLIP_PERCENT, LEP_I2C_COMMAND_TYPE_SET), percent);
}

uint16_t LeptonFLiR::agc_getHistogramClipPercent() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHistogramClipPercent"));
#endif

    uint16_t percent;
    receiveCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_CLIP_PERCENT, LEP_I2C_COMMAND_TYPE_GET), &percent);
    return percent;
}

void LeptonFLiR::agc_setHistogramTailSize(uint16_t size) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHistogramTailSize"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_TAIL_SIZE, LEP_I2C_COMMAND_TYPE_SET), size);
}

uint16_t LeptonFLiR::agc_getHistogramTailSize() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHistogramTailSize"));
#endif

    uint16_t size;
    receiveCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_TAIL_SIZE, LEP_I2C_COMMAND_TYPE_GET), &size);
    return size;
}

void LeptonFLiR::agc_setLinearMaxGain(uint16_t gain) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setLinearMaxGain"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_LINEAR_MAX_GAIN, LEP_I2C_COMMAND_TYPE_SET), gain);
}

uint16_t LeptonFLiR::agc_getLinearMaxGain() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getLinearMaxGain"));
#endif

    uint16_t gain;
    receiveCommand(cmdCode(LEP_CID_AGC_LINEAR_MAX_GAIN, LEP_I2C_COMMAND_TYPE_GET), &gain);
    return gain;
}

void LeptonFLiR::agc_setLinearMidpoint(uint16_t midpoint) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setLinearMidpoint"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_LINEAR_MIDPOINT, LEP_I2C_COMMAND_TYPE_SET), midpoint);
}

uint16_t LeptonFLiR::agc_getLinearMidpoint() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getLinearMidpoint"));
#endif

    uint16_t midpoint;
    receiveCommand(cmdCode(LEP_CID_AGC_LINEAR_MIDPOINT, LEP_I2C_COMMAND_TYPE_GET), &midpoint);
    return midpoint;
}

void LeptonFLiR::agc_setLinearDampeningFactor(uint16_t factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setLinearDampeningFactor"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_LINEAR_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_SET), factor);
}

uint16_t LeptonFLiR::agc_getLinearDampeningFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getLinearDampeningFactor"));
#endif

    uint16_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_LINEAR_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return factor;
}

void LeptonFLiR::agc_setHEQDampeningFactor(uint16_t factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHEQDampeningFactor"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_SET), factor);
}

uint16_t LeptonFLiR::agc_getHEQDampeningFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHEQDampeningFactor"));
#endif

    uint16_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return factor;
}

void LeptonFLiR::agc_setHEQMaxGain(uint16_t gain) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHEQMaxGain"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_MAX_GAIN, LEP_I2C_COMMAND_TYPE_SET), gain);
}

uint16_t LeptonFLiR::agc_getHEQMaxGain() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHEQMaxGain"));
#endif

    uint16_t gain;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_MAX_GAIN, LEP_I2C_COMMAND_TYPE_GET), &gain);
    return gain;
}

void LeptonFLiR::agc_setHEQClipLimitHigh(uint16_t limit) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHEQClipLimitHigh"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_HIGH, LEP_I2C_COMMAND_TYPE_SET), limit);
}

uint16_t LeptonFLiR::agc_getHEQClipLimitHigh() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHEQClipLimitHigh"));
#endif

    uint16_t limit;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_HIGH, LEP_I2C_COMMAND_TYPE_GET), &limit);
    return limit;
}

void LeptonFLiR::agc_setHEQClipLimitLow(uint16_t limit) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHEQClipLimitLow"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_LOW, LEP_I2C_COMMAND_TYPE_SET), limit);
}

uint16_t LeptonFLiR::agc_getHEQClipLimitLow() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHEQClipLimitLow"));
#endif

    uint16_t limit;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_LOW, LEP_I2C_COMMAND_TYPE_GET), &limit);
    return limit;
}

void LeptonFLiR::agc_setHEQBinExtension(uint16_t extension) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHEQBinExtension"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_BIN_EXTENSION, LEP_I2C_COMMAND_TYPE_SET), extension);
}

uint16_t LeptonFLiR::agc_getHEQBinExtension() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHEQBinExtension"));
#endif

    uint16_t extension;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_BIN_EXTENSION, LEP_I2C_COMMAND_TYPE_GET), &extension);
    return extension;
}

void LeptonFLiR::agc_setHEQMidpoint(uint16_t midpoint) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHEQMidpoint"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_MIDPOINT, LEP_I2C_COMMAND_TYPE_SET), midpoint);
}

uint16_t LeptonFLiR::agc_getHEQMidpoint() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHEQMidpoint"));
#endif

    uint16_t midpoint;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_MIDPOINT, LEP_I2C_COMMAND_TYPE_GET), &midpoint);
    return midpoint;
}

void LeptonFLiR::agc_setHEQEmptyCounts(uint16_t counts) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHEQEmptyCounts"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_EMPTY_COUNTS, LEP_I2C_COMMAND_TYPE_SET), counts);
}

uint16_t LeptonFLiR::agc_getHEQEmptyCounts() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHEQEmptyCounts"));
#endif

    uint16_t counts;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_EMPTY_COUNTS, LEP_I2C_COMMAND_TYPE_GET), &counts);
    return counts;
}

void LeptonFLiR::agc_setHEQNormalizationFactor(uint16_t factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_setHEQNormalizationFactor"));
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_NORMALIZATION_FACTOR, LEP_I2C_COMMAND_TYPE_SET), factor);
}

uint16_t LeptonFLiR::agc_getHEQNormalizationFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::agc_getHEQNormalizationFactor"));
#endif

    uint16_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_NORMALIZATION_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return factor;
}
