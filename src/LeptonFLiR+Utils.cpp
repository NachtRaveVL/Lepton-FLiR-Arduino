/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / Utilities
*/

#include "LeptonFLiR.h"

float LeptonFLiR::kelvin100ToTemperature(uint16_t kelvin100) {
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            return LeptonFLiR::kelvin100ToCelsius(kelvin100);
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            return LeptonFLiR::kelvin100ToFahrenheit(kelvin100);
        case LeptonFLiR_TemperatureMode_Kelvin:
            return LeptonFLiR::kelvin100ToKelvin(kelvin100);
        default:
            return 0;
    }
}

uint16_t LeptonFLiR::temperatureToKelvin100(float temperature) {
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            return LeptonFLiR::celsiusToKelvin100(temperature);
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            return LeptonFLiR::fahrenheitToKelvin100(temperature);
        case LeptonFLiR_TemperatureMode_Kelvin:
            return LeptonFLiR::kelvinToKelvin100(temperature);
        default:
            return 0;
    }
}

const char *LeptonFLiR::getTemperatureSymbol() {
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            return "°C";
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            return "°F";
        case LeptonFLiR_TemperatureMode_Kelvin:
            return "°K";
        default:
            return NULL;
    }
}

byte LeptonFLiR::getLastI2CError() {
    return _lastI2CError;
}

LEP_RESULT LeptonFLiR::getLastLepResult() {
    return (LEP_RESULT)_lastLepResult;
}

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

void LeptonFLiR::printModuleInfo() {
    char buffer[80];

    Serial.println(""); Serial.println(" ~~~ LeptonFLiR Module Info ~~~");

    Serial.println(""); Serial.println("Chip Select Pin:");
    Serial.print("D");
    Serial.print(_spiCSPin);
    Serial.println(" (active-low)");

    Serial.println(""); Serial.println("ISR VSync Pin:");
    Serial.print("D");
    Serial.print(_isrVSyncPin);
    if (_isrVSyncPin > 0)
        Serial.println(" (ISR-on-high)");
    else
        Serial.println(" (disabled)");

    Serial.println(""); Serial.println("SPI Port Speed:");
    const int divisor = getSPIClockDivisor();
    const float spiSpeed = F_CPU / (const float)divisor;
    Serial.print(roundf(spiSpeed / 1000.0f) / 1000.0f);
    Serial.print("MHz (SPI_CLOCK_DIV");
    Serial.print(divisor);
    Serial.print(")");
    if (spiSpeed < LEPFLIR_SPI_MIN_SPEED - FLT_EPSILON)
        Serial.println(" <speed too low>");
    else if (spiSpeed > LEPFLIR_SPI_MAX_SPEED + FLT_EPSILON)
        Serial.println(" <speed too high>");
    else if (spiSpeed < LEPFLIR_SPI_OPTIMAL_MIN_SPEED - FLT_EPSILON)
        Serial.println(" <speed sub-optimal>");
    else
        Serial.println("");

    Serial.println(""); Serial.println("Camera Type:");
    Serial.print(_cameraType);
    Serial.print(": ");
    switch (_cameraType) {
        case LeptonFLiR_CameraType_Lepton1:
            Serial.println("LeptonFLiR_CameraType_Lepton1"); break;
        case LeptonFLiR_CameraType_Lepton1_6:
            Serial.println("LeptonFLiR_CameraType_Lepton1_6"); break;
        case LeptonFLiR_CameraType_Lepton2:
            Serial.println("LeptonFLiR_CameraType_Lepton2"); break;
        case LeptonFLiR_CameraType_Lepton2_5:
            Serial.println("LeptonFLiR_CameraType_Lepton2_5"); break;
        case LeptonFLiR_CameraType_Lepton3:
            Serial.println("LeptonFLiR_CameraType_Lepton3"); break;
        case LeptonFLiR_CameraType_Lepton3_5:
            Serial.println("LeptonFLiR_CameraType_Lepton3_5"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Temperature Mode:");
    Serial.print(_tempMode);
    Serial.print(": ");
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            Serial.println("LeptonFLiR_TemperatureMode_Celsius"); break;
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            Serial.println("LeptonFLiR_TemperatureMode_Fahrenheit"); break;
        case LeptonFLiR_TemperatureMode_Kelvin:
            Serial.println("LeptonFLiR_TemperatureMode_Kelvin"); break;
        default:
            Serial.println(""); break;
    }

    LeptonFLiR::FrameSettings* nextFrame = getNextFrame();

    Serial.println(""); Serial.println("Image Storage Mode:");
    LeptonFLiR_ImageMode imageMode = nextFrame ? nextFrame->imageMode : LeptonFLiR_ImageMode_Undefined;
    Serial.print(imageMode);
    Serial.print(": ");
    switch (imageMode) {
        case LeptonFLiR_ImageMode_80x60_24bpp_244Brf:
            Serial.println("LeptonFLiR_ImageMode_80x60_24bpp_244Brf"); break;
        case LeptonFLiR_ImageMode_80x60_16bpp_164Brf:
            Serial.println("LeptonFLiR_ImageMode_80x60_16bpp_164Brf"); break;
        case LeptonFLiR_ImageMode_160x120_24bpp_244Brf:
            Serial.println("LeptonFLiR_ImageMode_160x120_24bpp_244Brf"); break;
        case LeptonFLiR_ImageMode_160x120_16bpp_164Brf:
            Serial.println("LeptonFLiR_ImageMode_160x120_16bpp_164Brf"); break;
        case LeptonFLiR_ImageMode_Undefined:
            Serial.println("LeptonFLiR_ImageMode_Undefined"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Image Output Mode:");
    LeptonFLiR_ImageOutputMode outputMode = nextFrame ? nextFrame->outputMode : LeptonFLiR_ImageOutputMode_Undefined;
    Serial.print(outputMode);
    Serial.print(": ");
    switch (outputMode) {
        case LeptonFLiR_ImageOutputMode_GS8:
            Serial.println("LeptonFLiR_ImageOutputMode_GS8"); break;
        case LeptonFLiR_ImageOutputMode_GS16:
            Serial.println("LeptonFLiR_ImageOutputMode_GS16"); break;
        case LeptonFLiR_ImageOutputMode_RGB888:
            Serial.println("LeptonFLiR_ImageOutputMode_RGB888"); break;
        case LeptonFLiR_ImageOutputMode_Undefined:
            Serial.println("LeptonFLiR_ImageOutputMode_Undefined"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Memory Footprint:");
    const int mallocOffset = 15;
    const int classSize = sizeof(LeptonFLiR);
    const int frameBufferSize = _frameData ? _frameDataSize_orig + mallocOffset : 0;
    const int imageOutputSize = _imageOutput ? _imageOutputSize_orig + mallocOffset : 0;
    const int telemetryOutputSize = _telemetryOutput ? sizeof(LeptonFLiR_TelemetryData) : 0;
    const int lastFrameSize = _lastFrame ? sizeof(LeptonFLiR::FrameSettings) + (_lastFrame->offsetTable ? getSPIFrameImageLines() * sizeof(uint16_t) : 0) : 0;
    const int nextFrameSize = _nextFrame ? sizeof(LeptonFLiR::FrameSettings) + (_nextFrame->offsetTable ? getSPIFrameImageLines() * sizeof(uint16_t) : 0) : 0;
    Serial.print("Class: ");
    Serial.print(classSize);
    Serial.print("B, raw buffer: ");
    Serial.print(frameBufferSize);
    Serial.print("B, image output: ");
    Serial.print(imageOutputSize);
    Serial.print("B, telemetry output: ");
    Serial.print(telemetryOutputSize);
    Serial.print("B, last frame: ");
    Serial.print(lastFrameSize);
    Serial.print("B, next frame: ");
    Serial.print(nextFrameSize);
    Serial.print("B, Total: ");
    Serial.print(classSize + frameBufferSize + imageOutputSize + telemetryOutputSize + lastFrameSize + nextFrameSize);
    Serial.println("B");

    Serial.println(""); Serial.println("Power Register:");
    uint16_t powerReg; readRegister(LEP_I2C_POWER_REG, &powerReg);
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
    Serial.print("0x");
    Serial.println(powerReg, HEX);

    Serial.println(""); Serial.println("Status Register:");
    uint16_t statusReg; readRegister(LEP_I2C_STATUS_REG, &statusReg);
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
    Serial.print("0x");
    Serial.println(statusReg, HEX);

    Serial.println(""); Serial.println("AGC Enabled:");
    Serial.println(agc_getAGCEnabled() ? "enabled" : "disabled");

    Serial.println(""); Serial.println("AGC Policy:");
    LEP_AGC_POLICY policy = agc_getAGCPolicy();
    Serial.print(policy);
    Serial.print(": ");
    switch (policy) {
        case LEP_AGC_LINEAR:
            Serial.println("LEP_AGC_LINEAR"); break;
        case LEP_AGC_HEQ:
            Serial.println("LEP_AGC_HEQ"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("AGC HEQ Scale Factor:");
    LEP_AGC_HEQ_SCALE_FACTOR factor = agc_getHEQScaleFactor();
    Serial.print(factor);
    Serial.print(": ");
    switch (factor) {
        case LEP_AGC_SCALE_TO_8_BITS:
            Serial.println("LEP_AGC_SCALE_TO_8_BITS"); break;
        case LEP_AGC_SCALE_TO_14_BITS:
            Serial.println("LEP_AGC_SCALE_TO_14_BITS"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("AGC Calculation Enabled:");
    Serial.println(agc_getAGCCalcEnabled() ? "enabled" : "disabled");

    Serial.println(""); Serial.println("SYS Camera Status:");
    LEP_SYS_CAM_STATUS_STATES camStatus = sys_getCameraStatus();
    Serial.print(camStatus);
    Serial.print(": ");
    switch (camStatus) {
        case LEP_SYSTEM_READY:
            Serial.println("LEP_SYSTEM_READY"); break;
        case LEP_SYSTEM_INITIALIZING:
            Serial.println("LEP_SYSTEM_INITIALIZING"); break;
        case LEP_SYSTEM_IN_LOW_POWER_MODE:
            Serial.println("LEP_SYSTEM_IN_LOW_POWER_MODE"); break;
        case LEP_SYSTEM_GOING_INTO_STANDBY:
            Serial.println("LEP_SYSTEM_GOING_INTO_STANDBY"); break;
        case LEP_SYSTEM_FLAT_FIELD_IN_PROCESS:
            Serial.println("LEP_SYSTEM_FLAT_FIELD_IN_PROCESS"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("FLiR Serial Number:");
    sys_getFlirSerialNumber(buffer, 80);
    Serial.println(buffer);

    Serial.println(""); Serial.println("Customer Serial Number:");
    sys_getCustomerSerialNumber(buffer, 80);
    Serial.println(buffer);

    Serial.println(""); Serial.println("Camera Uptime:");
    Serial.print(sys_getCameraUptime());
    Serial.println(" ms");

    Serial.println(""); Serial.println("Sys Aux Temperature:");
    Serial.print(sys_getAuxTemperature());
    Serial.println(getTemperatureSymbol());

    Serial.println(""); Serial.println("Sys FPA Temperature:");
    Serial.print(sys_getFPATemperature());
    Serial.println(getTemperatureSymbol());

    Serial.println(""); Serial.println("Telemetry Enabled:");
    Serial.println(sys_getTelemetryEnabled() ? "enabled" : "disabled");

    Serial.println(""); Serial.println("Vid Polarity:");
    LEP_VID_POLARITY polarity = vid_getPolarity();
    Serial.print(polarity);
    Serial.print(": ");
    switch (polarity) {
        case LEP_VID_WHITE_HOT:
            Serial.println("LEP_VID_WHITE_HOT"); break;
        case LEP_VID_BLACK_HOT:
            Serial.println("LEP_VID_BLACK_HOT"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Vid Pseudo Color Lookup Table:");
    LEP_VID_PCOLOR_LUT mode = vid_getPseudoColorLUT();
    Serial.print(mode);
    Serial.print(": ");
    switch (mode) {
        case LEP_VID_WHEEL6_LUT:
            Serial.println("LEP_VID_WHEEL6_LUT"); break;
        case LEP_VID_FUSION_LUT:
            Serial.println("LEP_VID_FUSION_LUT"); break;
        case LEP_VID_RAINBOW_LUT:
            Serial.println("LEP_VID_RAINBOW_LUT"); break;
        case LEP_VID_GLOBOW_LUT:
            Serial.println("LEP_VID_GLOBOW_LUT"); break;
        case LEP_VID_COLOR_LUT:
            Serial.println("LEP_VID_COLOR_LUT"); break;
        case LEP_VID_ICE_FIRE_LUT:
            Serial.println("LEP_VID_ICE_FIRE_LUT"); break;
        case LEP_VID_RAIN_LUT:
            Serial.println("LEP_VID_RAIN_LUT"); break;
        case LEP_VID_USER_LUT:
            Serial.println("LEP_VID_USER_LUT"); break;
        default:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Vid Focus Calculation Enabled:");
    Serial.println(vid_getFocusCalcEnabled() ? "enabled" : "disabled");

    Serial.println(""); Serial.println("Vid Freeze Enabled:");
    Serial.println(vid_getFreezeEnabled() ? "enabled" : "disabled");

    Serial.println(""); Serial.println("Vid Image Output Format:");
    LEP_VID_VIDEO_OUTPUT_FORMAT format = vid_getOutputFormat();
    Serial.print(format);
    Serial.print(": ");
    switch (mode) {
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RAW10:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RAW10"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RAW12:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RAW12"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RGB888:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RGB888"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RGB666:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RGB666"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RGB565:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RGB565"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_YUV422_8BIT:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_YUV422_8BIT"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_YUV422_10BIT:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_YUV422_10BIT"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_USER_DEFINED:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_USER_DEFINED"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_2:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_2"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_3:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_3"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_4:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_4"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_5:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_5"); break;
        case LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_6:
            Serial.println("LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_6"); break;
        default:
            Serial.println(""); break;
    }

    // TODO: Add basic OEM module outputs. -NR

    // TODO: Add basic RAD module outputs. -NR
}

static const char *textForI2CError(byte errorCode) {
    switch (errorCode) {
        case 0:
            return "Success";
        case 1:
            return "Data too long to fit in transmit buffer";
        case 2:
            return "Received NACK on transmit of address";
        case 3:
            return "Received NACK on transmit of data";
        default:
            return "Other error";
    }
}

static const char *textForLepResult(LEP_RESULT errorCode) {
    switch (errorCode) {
        case LEP_OK:
            return "LEP_OK Camera ok";
        case LEP_ERROR:
            return "LEP_ERROR Camera general error";
        case LEP_NOT_READY:
            return "LEP_NOT_READY Camera not ready error";
        case LEP_RANGE_ERROR:
            return "LEP_RANGE_ERROR Camera range error";
        case LEP_CHECKSUM_ERROR:
            return "LEP_CHECKSUM_ERROR Camera checksum error";
        case LEP_BAD_ARG_POINTER_ERROR:
            return "LEP_BAD_ARG_POINTER_ERROR Camera Bad argument  error";
        case LEP_DATA_SIZE_ERROR:
            return "LEP_DATA_SIZE_ERROR Camera byte count error";
        case LEP_UNDEFINED_FUNCTION_ERROR:
            return "LEP_UNDEFINED_FUNCTION_ERROR Camera undefined function error";
        case LEP_FUNCTION_NOT_SUPPORTED:
            return "LEP_FUNCTION_NOT_SUPPORTED Camera function not yet supported error";
        case LEP_OTP_WRITE_ERROR:
            return "LEP_OTP_WRITE_ERROR Camera OTP write error";
        case LEP_OTP_READ_ERROR:
            return "LEP_OTP_READ_ERROR Double bit error detected (uncorrectible)";
        case LEP_OTP_NOT_PROGRAMMED_ERROR:
            return "LEP_OTP_NOT_PROGRAMMED_ERROR Flag read as non-zero";
        case LEP_ERROR_I2C_BUS_NOT_READY:
            return "LEP_ERROR_I2C_BUS_NOT_READY I2C Bus Error - Bus Not Avaialble";
        case LEP_ERROR_I2C_BUFFER_OVERFLOW:
            return "LEP_ERROR_I2C_BUFFER_OVERFLOW I2C Bus Error - Buffer Overflow";
        case LEP_ERROR_I2C_ARBITRATION_LOST:
            return "LEP_ERROR_I2C_ARBITRATION_LOST I2C Bus Error - Bus Arbitration Lost";
        case LEP_ERROR_I2C_BUS_ERROR:
            return "LEP_ERROR_I2C_BUS_ERROR I2C Bus Error - General Bus Error";
        case LEP_ERROR_I2C_NACK_RECEIVED:
            return "LEP_ERROR_I2C_NACK_RECEIVED I2C Bus Error - NACK Received";
        case LEP_ERROR_I2C_FAIL:
            return "LEP_ERROR_I2C_FAIL I2C Bus Error - General Failure";
        case LEP_DIV_ZERO_ERROR:
            return "LEP_DIV_ZERO_ERROR Attempted div by zero";
        case LEP_COMM_PORT_NOT_OPEN:
            return "LEP_COMM_PORT_NOT_OPEN Comm port not open";
        case LEP_COMM_INVALID_PORT_ERROR:
            return "LEP_COMM_INVALID_PORT_ERROR Comm port no such port error";
        case LEP_COMM_RANGE_ERROR:
            return "LEP_COMM_RANGE_ERROR Comm port range error";
        case LEP_ERROR_CREATING_COMM:
            return "LEP_ERROR_CREATING_COMM Error creating comm";
        case LEP_ERROR_STARTING_COMM:
            return "LEP_ERROR_STARTING_COMM Error starting comm";
        case LEP_ERROR_CLOSING_COMM:
            return "LEP_ERROR_CLOSING_COMM Error closing comm";
        case LEP_COMM_CHECKSUM_ERROR:
            return "LEP_COMM_CHECKSUM_ERROR Comm checksum error";
        case LEP_COMM_NO_DEV:
            return "LEP_COMM_NO_DEV No comm device";
        case LEP_TIMEOUT_ERROR:
            return "LEP_TIMEOUT_ERROR Comm timeout error";
        case LEP_COMM_ERROR_WRITING_COMM:
            return "LEP_COMM_ERROR_WRITING_COMM Error writing comm";
        case LEP_COMM_ERROR_READING_COMM:
            return "LEP_COMM_ERROR_READING_COMM Error reading comm";
        case LEP_COMM_COUNT_ERROR:
            return "LEP_COMM_COUNT_ERROR Comm byte count error";
        case LEP_OPERATION_CANCELED:
            return "LEP_OPERATION_CANCELED Camera operation canceled";
        case LEP_UNDEFINED_ERROR_CODE:
            return "LEP_UNDEFINED_ERROR_CODE Undefined error";
        default:
            return "Other error";
    }
}

void LeptonFLiR::checkForErrors() {
    if (_lastI2CError) {
        Serial.print("  LeptonFLiR::checkErrors lastI2CError: ");
        Serial.print(_lastI2CError);
        Serial.print(": ");
        Serial.println(textForI2CError(getLastI2CError()));
    }
    if (_lastLepResult) {
        Serial.print("  LeptonFLiR::checkErrors lastLepResult: ");
        Serial.print(_lastLepResult);
        Serial.print(": ");
        Serial.println(textForLepResult(getLastLepResult()));
    }
}

#endif // /#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

static inline void byteToHexString(byte value, char *buffer) {
    byte highNibble = value / 16;
    byte lowNibble = value % 16;
    if (highNibble < 10) buffer[0] = '0' + highNibble;
    else buffer[0] = 'A' + (highNibble - 10);
    if (lowNibble < 10) buffer[1] = '0' + lowNibble;
    else buffer[1] = 'A' + (lowNibble - 10);
}

void LeptonFLiR::wordsToHexString(uint16_t *dataWords, int dataLength, char *buffer, int maxLength) {
    bool insertColons = maxLength >= (dataLength * 4) + (dataLength - 1);

    while (dataLength-- > 0 && maxLength > 3) {
        if (maxLength > 3) {
            byteToHexString(highByte(*dataWords), buffer);
            buffer += 2; maxLength -= 2;
            byteToHexString(lowByte(*dataWords), buffer);
            buffer += 2; maxLength -= 2;
            ++dataWords;
        }

        if (dataLength > 0 && insertColons && maxLength-- > 0)
            *buffer++ = ':';
    }

    if (maxLength-- > 0)
        *buffer++ = '\0';
}
