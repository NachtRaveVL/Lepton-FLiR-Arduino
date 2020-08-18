/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / AGC Module Commands
*/

#ifndef LeptonFLiR_AGC_H
#define LeptonFLiR_AGC_H

#include "LeptonFLiR.h"

#ifndef LEPFLIR_IN_PUBLIC
#error "This file not meant to be directly included. Include LeptonFLiR.h instead."
#endif
// class LeptonFLiR {
// public:

    void agc_setAGCEnabled(bool enabled); // def:disabled
    bool agc_getAGCEnabled();

    void agc_setAGCPolicy(LEP_AGC_POLICY policy); // def:LEP_AGC_HEQ
    LEP_AGC_POLICY agc_getAGCPolicy();

    void agc_setHEQScaleFactor(LEP_AGC_HEQ_SCALE_FACTOR factor); // def:LEP_AGC_SCALE_TO_8_BITS
    LEP_AGC_HEQ_SCALE_FACTOR agc_getHEQScaleFactor();

    void agc_setAGCCalcEnabled(bool enabled); // def:disabled
    bool agc_getAGCCalcEnabled();

    // TODO: Check to make sure we have all the AGC module commands up to v3.5. -NR

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

#endif // /#ifndef LEPFLIR_EXCLUDE_EXT_I2C_FUNCS

#endif // /#ifndef LeptonFLiR_AGC_H
