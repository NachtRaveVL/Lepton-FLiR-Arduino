/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / VID Module Commands
*/
#ifndef LeptonFLiR_VID_H
#define LeptonFLiR_VID_H

#include "LeptonFLiR.h"

#ifndef LEPFLIR_IN_PUBLIC
#error "This file not meant to be directly included. Include LeptonFLiR.h instead."
#endif
// class LeptonFLiR {
// public:

    // VID module commands

    void vid_setPolarity(LEP_VID_POLARITY polarity); // def:LEP_VID_WHITE_HOT
    LEP_VID_POLARITY vid_getPolarity();

    void vid_setPseudoColorLUT(LEP_VID_PCOLOR_LUT mode); // def:LEP_VID_FUSION_LUT
    LEP_VID_PCOLOR_LUT vid_getPseudoColorLUT(); 

    void vid_setFocusCalcEnabled(bool enabled); // def:disabled
    bool vid_getFocusCalcEnabled();

    void vid_setFreezeEnabled(bool enabled); // def:disabled
    bool vid_getFreezeEnabled();

    void vid_setOutputFormat(LEP_VID_VIDEO_OUTPUT_FORMAT format); // def:LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14
    LEP_VID_VIDEO_OUTPUT_FORMAT vid_getOutputFormat();

    // TODO: Check to make sure we have all the VID module commands up to v3.5. -NR

    void vid_setUserColorLUT(LEP_VID_LUT_BUFFER *mode); // These two methods may not work as intended, possibly leaving the I2C bus on the
    void vid_getUserColorLUT(LEP_VID_LUT_BUFFER *mode); // FLiR in a non-responding state. A full power cycle may be needed to reset.

    void vid_setFocusRegion(LEP_VID_FOCUS_ROI *region); // min:1,1/end>beg+1, max:78,58/beg<end-1 def:{1,1,78,58} (pixels)
    void vid_getFocusRegion(LEP_VID_FOCUS_ROI *region);

    void vid_setFocusThreshold(uint32_t threshold); // def:30
    uint32_t vid_getFocusThreshold();

    uint32_t vid_getFocusMetric();

    void vid_setSceneBasedNUCEnabled(bool enabled); // def:enabled
    bool vid_getSceneBasedNUCEnabled();

    void vid_setGamma(uint32_t gamma); // def:58
    uint32_t vid_getGamma();

#endif // /ifndef LeptonFLiR_VID_H
