/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / OEM Module Commands
*/

#include "LeptonFLiR.h"

namespace {

void oemWordsToString(const uint16_t *words, int wordCount, char *buffer, int maxLength) {
    if (!words || !buffer || maxLength <= 0)
        return;

    int out = 0;
    for (int i = 0; i < wordCount && out < maxLength - 1; ++i) {
        buffer[out++] = (char)highByte(words[i]);
        if (out < maxLength - 1)
            buffer[out++] = (char)lowByte(words[i]);
    }
    buffer[out] = '\0';
}

uint32_t oemState(bool enabled) {
    return enabled ? 1u : 0u;
}

} // namespace

void LeptonFLiR::oem_runPowerOn() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_runPowerOn"));
#endif

    // This is an SDK-side operation rather than an OEM command word.
    // After OEM power-down, the Lepton also requires the host to free SDA with
    // one SCL pulse before this register write can wake the module.
    writeRegister(LEP_I2C_POWER_REG, 0x0000);
}

void LeptonFLiR::oem_runPowerDown() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_runPowerDown"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_POWER_MODE, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::oem_getFlirPartNumber(char *buffer, int maxLength) {
    if (!buffer || maxLength <= 0) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getFlirPartNumber"));
#endif

    uint16_t words[16] = {};
    receiveCommand(cmdCode(LEP_CID_OEM_FLIR_PART_NUMBER, LEP_I2C_COMMAND_TYPE_GET), words, 16);
    oemWordsToString(words, 16, buffer, maxLength);
}

void LeptonFLiR::oem_getSoftwareVersion(LEP_OEM_SW_VERSION *version) {
    if (!version) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getSoftwareVersion"));
#endif

    uint16_t words[4] = {};
    receiveCommand(cmdCode(LEP_CID_OEM_SOFTWARE_REVISION, LEP_I2C_COMMAND_TYPE_GET), words, 4);
    version->gpp_major = highByte(words[0]);
    version->gpp_minor = lowByte(words[0]);
    version->gpp_build = highByte(words[1]);
    version->dsp_major = lowByte(words[1]);
    version->dsp_minor = highByte(words[2]);
    version->dsp_build = lowByte(words[2]);
    version->reserved = words[3];
}

void LeptonFLiR::oem_setVideoOutputEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setVideoOutputEnabled"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_VID_OUTPUT_ENABLE, LEP_I2C_COMMAND_TYPE_SET), oemState(enabled));
}

bool LeptonFLiR::oem_getVideoOutputEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getVideoOutputEnabled"));
#endif

    uint32_t enabled = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_VID_OUTPUT_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled != 0;
}

void LeptonFLiR::oem_setVideoOutputFormat(LEP_OEM_VIDEO_OUTPUT_FORMAT format) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setVideoOutputFormat"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_VID_OUTPUT_FORMAT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)format);
    _nextFrameNeedsUpdate = true;
}

LEP_OEM_VIDEO_OUTPUT_FORMAT LeptonFLiR::oem_getVideoOutputFormat() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getVideoOutputFormat"));
#endif

    uint32_t format = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_VID_OUTPUT_FORMAT, LEP_I2C_COMMAND_TYPE_GET), &format);
    return (LEP_OEM_VIDEO_OUTPUT_FORMAT)format;
}

void LeptonFLiR::oem_setVideoOutputSource(LEP_OEM_VIDEO_OUTPUT_SOURCE source) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setVideoOutputSource"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_VID_OUTPUT_SOURCE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)source);
}

LEP_OEM_VIDEO_OUTPUT_SOURCE LeptonFLiR::oem_getVideoOutputSource() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getVideoOutputSource"));
#endif

    uint32_t source = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_VID_OUTPUT_SOURCE, LEP_I2C_COMMAND_TYPE_GET), &source);
    return (LEP_OEM_VIDEO_OUTPUT_SOURCE)source;
}

void LeptonFLiR::oem_getCustomerPartNumber(char *buffer, int maxLength) {
    if (!buffer || maxLength <= 0) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getCustomerPartNumber"));
#endif

    uint16_t words[16] = {};
    receiveCommand(cmdCode(LEP_CID_OEM_CUST_PART_NUMBER, LEP_I2C_COMMAND_TYPE_GET), words, 16);
    oemWordsToString(words, 16, buffer, maxLength);
}

void LeptonFLiR::oem_setVideoOutputSourceConstant(uint16_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setVideoOutputSourceConstant"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_OUTPUT_SOURCE_CONST, LEP_I2C_COMMAND_TYPE_SET), value);
}

uint16_t LeptonFLiR::oem_getVideoOutputSourceConstant() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getVideoOutputSourceConstant"));
#endif

    uint16_t value = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_OUTPUT_SOURCE_CONST, LEP_I2C_COMMAND_TYPE_GET), &value);
    return value;
}

void LeptonFLiR::oem_runReboot() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_runReboot"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_CAMERA_REBOOT, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::oem_setFFCNormalizationTarget(uint16_t target) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setFFCNormalizationTarget"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_FFC_NORM_TARGET, LEP_I2C_COMMAND_TYPE_SET), target);
}

uint16_t LeptonFLiR::oem_getFFCNormalizationTarget() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getFFCNormalizationTarget"));
#endif

    uint16_t target = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_FFC_NORM_TARGET, LEP_I2C_COMMAND_TYPE_GET), &target);
    return target;
}

void LeptonFLiR::oem_runFFCNormalization() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_runFFCNormalization"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_FFC_NORM_TARGET, LEP_I2C_COMMAND_TYPE_RUN));
}

LEP_OEM_STATUS LeptonFLiR::oem_getStatus() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getStatus"));
#endif

    uint32_t status = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_STATUS, LEP_I2C_COMMAND_TYPE_GET), &status);
    return (LEP_OEM_STATUS)status;
}

uint16_t LeptonFLiR::oem_getFrameMean() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getFrameMean"));
#endif

    uint16_t mean = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_FRAME_MEAN, LEP_I2C_COMMAND_TYPE_GET), &mean);
    return mean;
}

void LeptonFLiR::oem_setGPIOMode(LEP_OEM_GPIO_MODE mode) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setGPIOMode"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_GPIO_MODE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)mode);
}

LEP_OEM_GPIO_MODE LeptonFLiR::oem_getGPIOMode() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getGPIOMode"));
#endif

    uint32_t mode = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_GPIO_MODE, LEP_I2C_COMMAND_TYPE_GET), &mode);
    return (LEP_OEM_GPIO_MODE)mode;
}

void LeptonFLiR::oem_setVSyncDelay(LEP_OEM_VSYNC_DELAY delay) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setVSyncDelay"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_VSYNC_DELAY, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)(int32_t)delay);
}

LEP_OEM_VSYNC_DELAY LeptonFLiR::oem_getVSyncDelay() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getVSyncDelay"));
#endif

    uint32_t delay = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_VSYNC_DELAY, LEP_I2C_COMMAND_TYPE_GET), &delay);
    return (LEP_OEM_VSYNC_DELAY)(int32_t)delay;
}

LEP_OEM_USER_PARAMS_STATE LeptonFLiR::oem_getUserDefaultsState() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getUserDefaultsState"));
#endif

    uint32_t state = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_USER_PARAMS_STATE, LEP_I2C_COMMAND_TYPE_GET), &state);
    return (LEP_OEM_USER_PARAMS_STATE)state;
}

void LeptonFLiR::oem_runUserDefaultsCopyToOTP() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_runUserDefaultsCopyToOTP"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_USER_PARAMS_STATE, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::oem_runUserDefaultsRestore() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_runUserDefaultsRestore"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_USER_PARAMS_RESTORE, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::oem_setShutterProfile(LEP_OEM_SHUTTER_PROFILE *profile) {
    if (!profile) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setShutterProfile"));
#endif

    sendCommand(cmdCode(LEP_CID_OEM_SHUTTER_PROFILE, LEP_I2C_COMMAND_TYPE_SET),
                (uint16_t *)profile, sizeof(LEP_OEM_SHUTTER_PROFILE) / 2);
}

void LeptonFLiR::oem_getShutterProfile(LEP_OEM_SHUTTER_PROFILE *profile) {
    if (!profile) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getShutterProfile"));
#endif

    receiveCommand(cmdCode(LEP_CID_OEM_SHUTTER_PROFILE, LEP_I2C_COMMAND_TYPE_GET),
                   (uint16_t *)profile, sizeof(LEP_OEM_SHUTTER_PROFILE) / 2);
}

void LeptonFLiR::oem_setThermalShutdownEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setThermalShutdownEnabled"));
#endif
    sendCommand(cmdCode(LEP_CID_OEM_THERM_SHUTDOWN_ENABLE, LEP_I2C_COMMAND_TYPE_SET), oemState(enabled));
}

bool LeptonFLiR::oem_getThermalShutdownEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getThermalShutdownEnabled"));
#endif
    uint32_t enabled = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_THERM_SHUTDOWN_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled != 0;
}

void LeptonFLiR::oem_setBadPixelReplacementEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setBadPixelReplacementEnabled"));
#endif
    sendCommand(cmdCode(LEP_CID_OEM_BAD_PX_REPLACE_ENABLE, LEP_I2C_COMMAND_TYPE_SET), oemState(enabled));
}

bool LeptonFLiR::oem_getBadPixelReplacementEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getBadPixelReplacementEnabled"));
#endif
    uint32_t enabled = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_BAD_PX_REPLACE_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled != 0;
}

void LeptonFLiR::oem_setTemporalFilterEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setTemporalFilterEnabled"));
#endif
    sendCommand(cmdCode(LEP_CID_OEM_TEMPORAL_FILTER_ENABLE, LEP_I2C_COMMAND_TYPE_SET), oemState(enabled));
}

bool LeptonFLiR::oem_getTemporalFilterEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getTemporalFilterEnabled"));
#endif
    uint32_t enabled = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_TEMPORAL_FILTER_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled != 0;
}

void LeptonFLiR::oem_setColumnNoiseFilterEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setColumnNoiseFilterEnabled"));
#endif
    sendCommand(cmdCode(LEP_CID_OEM_COL_NOISE_FILTER_ENABLE, LEP_I2C_COMMAND_TYPE_SET), oemState(enabled));
}

bool LeptonFLiR::oem_getColumnNoiseFilterEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getColumnNoiseFilterEnabled"));
#endif
    uint32_t enabled = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_COL_NOISE_FILTER_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled != 0;
}

void LeptonFLiR::oem_setPixelNoiseFilterEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_setPixelNoiseFilterEnabled"));
#endif
    sendCommand(cmdCode(LEP_CID_OEM_PX_NOISE_FILTER_ENABLE, LEP_I2C_COMMAND_TYPE_SET), oemState(enabled));
}

bool LeptonFLiR::oem_getPixelNoiseFilterEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("LeptonFLiR::oem_getPixelNoiseFilterEnabled"));
#endif
    uint32_t enabled = 0;
    receiveCommand(cmdCode(LEP_CID_OEM_PX_NOISE_FILTER_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled != 0;
}
