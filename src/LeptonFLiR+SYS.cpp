/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / SYS Module Commands
*/

#include "LeptonFLiR.h"

void LeptonFLiR::sys_getCameraStatus(LEP_SYS_CAM_STATUS *status) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getCameraStatus");
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_CAM_STATUS, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)status, sizeof(LEP_SYS_CAM_STATUS) / 2);
}

LEP_SYS_CAM_STATUS_STATES LeptonFLiR::sys_getCameraStatus() {
    LEP_SYS_CAM_STATUS camStatus;
    sys_getCameraStatus(&camStatus);
    return (LEP_SYS_CAM_STATUS_STATES)camStatus.camStatus;
}

void LeptonFLiR::sys_getFlirSerialNumber(char *buffer, int maxLength) {
    if (!buffer || maxLength < 16) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getFlirSerialNumber");
#endif

    uint16_t innerBuffer[4];
    receiveCommand(cmdCode(LEP_CID_SYS_FLIR_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), innerBuffer, 4);
    LeptonFLiR::wordsToHexString(innerBuffer, 4, buffer, maxLength);
}

void LeptonFLiR::sys_getCustomerSerialNumber(char *buffer, int maxLength) {
    if (!buffer || maxLength < 64) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getCustomerSerialNumber");
#endif

    uint16_t innerBuffer[16];
    receiveCommand(cmdCode(LEP_CID_SYS_CUST_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), innerBuffer, 16);
    LeptonFLiR::wordsToHexString(innerBuffer, 16, buffer, maxLength);
}

uint32_t LeptonFLiR::sys_getCameraUptime() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getCameraUptime");
#endif

    uint32_t uptime;
    receiveCommand(cmdCode(LEP_CID_SYS_CAM_UPTIME, LEP_I2C_COMMAND_TYPE_GET), &uptime);
    return uptime;
}

float LeptonFLiR::sys_getAuxTemperature() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getAuxTemperature");
#endif

    uint16_t kelvin100;
    receiveCommand(cmdCode(LEP_CID_SYS_AUX_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), &kelvin100);
    return kelvin100ToTemperature(kelvin100);
}

float LeptonFLiR::sys_getFPATemperature() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getFPATemperature");
#endif

    uint16_t kelvin100;
    receiveCommand(cmdCode(LEP_CID_SYS_FPA_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), &kelvin100);
    return kelvin100ToTemperature(kelvin100);
}

void LeptonFLiR::sys_setTelemetryEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setTelemetryEnabled");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::sys_getTelemetryEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getTelemetryEnabled");
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::sys_setTelemetryLocation(LEP_SYS_TELEMETRY_LOCATION location) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setTelemetryLocation");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_TELEMETRY_LOCATION, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)location);
}

LEP_SYS_TELEMETRY_LOCATION LeptonFLiR::sys_getTelemetryLocation() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getTelemetryLocation");
#endif

    uint32_t location;
    receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_LOCATION, LEP_I2C_COMMAND_TYPE_GET), &location);
    return (LEP_SYS_TELEMETRY_LOCATION)location;
}

void LeptonFLiR::sys_runFFCNormalization() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_runFFCNormalization");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_RUN_FFC, LEP_I2C_COMMAND_TYPE_RUN));
}

#ifndef LEPFLIR_EXCLUDE_EXT_I2C_FUNCS

void LeptonFLiR::sys_runPingCamera() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_runPingCamera");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_PING, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::sys_runFrameAveraging() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_runFrameAveraging");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_EXECTUE_FRAME_AVERAGE, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::sys_setNumFramesToAverage(LEP_SYS_FRAME_AVERAGE average) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setNumFramesToAverage");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)average);
}

LEP_SYS_FRAME_AVERAGE LeptonFLiR::sys_getNumFramesToAverage() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getNumFramesToAverage");
#endif

    uint32_t average;
    receiveCommand(cmdCode(LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE, LEP_I2C_COMMAND_TYPE_GET), &average);
    return (LEP_SYS_FRAME_AVERAGE)average;
}

void LeptonFLiR::sys_getSceneStatistics(LEP_SYS_SCENE_STATISTICS *statistics) {
    if (!statistics) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getSceneStatistics");
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_SCENE_STATISTICS, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)statistics, sizeof(LEP_SYS_SCENE_STATISTICS) / 2);
}

void LeptonFLiR::sys_setSceneRegion(LEP_SYS_SCENE_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setSceneRegion");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_SCENE_ROI, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)region, sizeof(LEP_SYS_SCENE_ROI) / 2);
}

void LeptonFLiR::sys_getSceneRegion(LEP_SYS_SCENE_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getSceneRegion");
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_SCENE_ROI, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)region, sizeof(LEP_SYS_SCENE_ROI) / 2);
}

uint16_t LeptonFLiR::sys_getThermalShutdownCount() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getThermalShutdownCount");
#endif

    uint16_t count;
    receiveCommand(cmdCode(LEP_CID_SYS_THERMAL_SHUTDOWN_COUNT, LEP_I2C_COMMAND_TYPE_GET), &count);
    return count;
}

void LeptonFLiR::sys_setShutterPosition(LEP_SYS_SHUTTER_POSITION position) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setShutterPosition");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_SHUTTER_POSITION, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)position);
}

LEP_SYS_SHUTTER_POSITION LeptonFLiR::sys_getShutterPosition() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getShutterPosition");
#endif

    uint32_t position;
    receiveCommand(cmdCode(LEP_CID_SYS_SHUTTER_POSITION, LEP_I2C_COMMAND_TYPE_GET), &position);
    return (LEP_SYS_SHUTTER_POSITION)position;
}

void LeptonFLiR::sys_setFFCShutterMode(LEP_SYS_FFC_SHUTTER_MODE *mode) {
    if (!mode) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_setFFCShutterMode");
#endif

    sendCommand(cmdCode(LEP_CID_SYS_FFC_SHUTTER_MODE, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)mode, sizeof(LEP_SYS_FFC_SHUTTER_MODE) / 2);
}

void LeptonFLiR::sys_getFFCShutterMode(LEP_SYS_FFC_SHUTTER_MODE *mode) {
    if (!mode) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getFFCShutterMode");
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_FFC_SHUTTER_MODE, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)mode, sizeof(LEP_SYS_FFC_SHUTTER_MODE) / 2);
}

LEP_SYS_FFC_STATUS LeptonFLiR::sys_getFFCNormalizationStatus() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("LeptonFLiR::sys_getFFCNormalizationStatus");
#endif

    uint32_t status;
    receiveCommand(cmdCode(LEP_CID_SYS_FFC_STATUS, LEP_I2C_COMMAND_TYPE_GET), &status);
    return (LEP_SYS_FFC_STATUS)status;
}

#endif // /#ifndef LEPFLIR_EXCLUDE_EXT_I2C_FUNCS
