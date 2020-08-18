/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016 NachtRaveVL      <nachtravevl@gmail.com>
    LeptonFLiR Interface Description Document (IDD) Defines
*/

// The majority of this file has been cherry picked from the Lepton FLiR
// development SDK. To maintain consistency with the software interface
// description document, the following copyright notice is hearby included:

/*******************************************************************************
**
**      Copyright 2011,2012,2013,2014 FLIR Systems - Commercial
**      Vision Systems.  All rights reserved.
**
**      Proprietary - PROPRIETARY - FLIR Systems Inc..
**
**      This document is controlled to FLIR Technology Level 2.
**      The information contained in this document pertains to a
**      dual use product Controlled for export by the Export
**      Administration Regulations (EAR). Diversion contrary to
**      US law is prohibited.  US Department of Commerce
**      authorization is not required prior to export or
**      transfer to foreign persons or parties unless otherwise
**      prohibited.
**
**      Redistribution and use in source and binary forms, with
**      or without modification, are permitted provided that the
**      following conditions are met:
**
**      Redistributions of source code must retain the above
**      copyright notice, this list of conditions and the
**      following disclaimer.
**
**      Redistributions in binary form must reproduce the above
**      copyright notice, this list of conditions and the
**      following disclaimer in the documentation and/or other
**      materials provided with the distribution.
**
**      Neither the name of the FLIR Systems Corporation nor the
**      names of its contributors may be used to endorse or
**      promote products derived from this software without
**      specific prior written permission.
**
**      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
**      CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
**      WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**      WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
**      PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
**      COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY
**      DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
**      CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
**      PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
**      USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
**      CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
**      CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
**      NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
**      USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
**      OF SUCH DAMAGE.
**
*******************************************************************************/

#ifndef LeptonFLiRIDDDefs_H
#define LeptonFLiRIDDDefs_H

// i2c

#define LEP_I2C_DEVICE_ADDRESS                  (byte)0x2A

#define LEP_I2C_COMMAND_MODULE_ID_BIT_MASK      (uint16_t)0x0F00
#define LEP_I2C_COMMAND_ID_BIT_MASK             (uint16_t)0x00FC
#define LEP_I2C_COMMAND_TYPE_BIT_MASK           (uint16_t)0x0003

#define LEP_I2C_COMMAND_TYPE_GET                (uint16_t)0x0000
#define LEP_I2C_COMMAND_TYPE_SET                (uint16_t)0x0001
#define LEP_I2C_COMMAND_TYPE_RUN                (uint16_t)0x0002
#define LEP_I2C_COMMAND_PROT_BIT                (uint16_t)0x4000    // For OEM and RAD modules only

#define LEP_I2C_STATUS_BUSY_BIT_MASK            (uint16_t)0x0001
#define LEP_I2C_STATUS_BOOT_MODE_BIT_MASK       (uint16_t)0x0002
#define LEP_I2C_STATUS_BOOT_STATUS_BIT_MASK     (uint16_t)0x0004
#define LEP_I2C_STATUS_ERROR_CODE_BIT_MASK      (uint16_t)0xFF00
#define LEP_I2C_STATUS_ERROR_CODE_BIT_SHIFT     8

#define LEP_I2C_REG_BASE_ADDR                   (uint16_t)0x0000
#define LEP_I2C_POWER_REG                       (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0000)
#define LEP_I2C_STATUS_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0002)
#define LEP_I2C_COMMAND_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0004)
#define LEP_I2C_DATA_LENGTH_REG                 (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0006)
#define LEP_I2C_DATA_0_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0008)
#define LEP_I2C_DATA_1_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x000A)
#define LEP_I2C_DATA_2_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x000C)
#define LEP_I2C_DATA_3_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x000E)
#define LEP_I2C_DATA_4_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0010)
#define LEP_I2C_DATA_5_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0012)
#define LEP_I2C_DATA_6_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0014)
#define LEP_I2C_DATA_7_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0016)
#define LEP_I2C_DATA_8_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0018)
#define LEP_I2C_DATA_9_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x001A)
#define LEP_I2C_DATA_10_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x001C)
#define LEP_I2C_DATA_11_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x001E)
#define LEP_I2C_DATA_12_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0020)
#define LEP_I2C_DATA_13_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0022)
#define LEP_I2C_DATA_14_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0024)
#define LEP_I2C_DATA_15_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0026)
#define LEP_I2C_DATA_CRC_REG                    (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0028)

#define LEP_I2C_DATA_BUFFER                     (uint16_t)0xF800
#define LEP_I2C_DATA_BUFFER_LENGTH              (uint16_t)0x0800

// AGC

#define LEP_AGC_MODULE_BASE                     (uint16_t)0x0100
#define LEP_CID_AGC_ENABLE_STATE                (uint16_t)(LEP_AGC_MODULE_BASE + 0x0000)
#define LEP_CID_AGC_POLICY                      (uint16_t)(LEP_AGC_MODULE_BASE + 0x0004)
#define LEP_CID_AGC_ROI                         (uint16_t)(LEP_AGC_MODULE_BASE + 0x0008)
#define LEP_CID_AGC_STATISTICS                  (uint16_t)(LEP_AGC_MODULE_BASE + 0x000C)
#define LEP_CID_AGC_HISTOGRAM_CLIP_PERCENT      (uint16_t)(LEP_AGC_MODULE_BASE + 0x0010)
#define LEP_CID_AGC_HISTOGRAM_TAIL_SIZE         (uint16_t)(LEP_AGC_MODULE_BASE + 0x0014)
#define LEP_CID_AGC_LINEAR_MAX_GAIN             (uint16_t)(LEP_AGC_MODULE_BASE + 0x0018)
#define LEP_CID_AGC_LINEAR_MIDPOINT             (uint16_t)(LEP_AGC_MODULE_BASE + 0x001C)
#define LEP_CID_AGC_LINEAR_DAMPENING_FACTOR     (uint16_t)(LEP_AGC_MODULE_BASE + 0x0020)
#define LEP_CID_AGC_HEQ_DAMPENING_FACTOR        (uint16_t)(LEP_AGC_MODULE_BASE + 0x0024)
#define LEP_CID_AGC_HEQ_MAX_GAIN                (uint16_t)(LEP_AGC_MODULE_BASE + 0x0028)
#define LEP_CID_AGC_HEQ_CLIP_LIMIT_HIGH         (uint16_t)(LEP_AGC_MODULE_BASE + 0x002C)
#define LEP_CID_AGC_HEQ_CLIP_LIMIT_LOW          (uint16_t)(LEP_AGC_MODULE_BASE + 0x0030)
#define LEP_CID_AGC_HEQ_BIN_EXTENSION           (uint16_t)(LEP_AGC_MODULE_BASE + 0x0034)
#define LEP_CID_AGC_HEQ_MIDPOINT                (uint16_t)(LEP_AGC_MODULE_BASE + 0x0038)
#define LEP_CID_AGC_HEQ_EMPTY_COUNTS            (uint16_t)(LEP_AGC_MODULE_BASE + 0x003C)
#define LEP_CID_AGC_HEQ_NORMALIZATION_FACTOR    (uint16_t)(LEP_AGC_MODULE_BASE + 0x0040)
#define LEP_CID_AGC_HEQ_SCALE_FACTOR            (uint16_t)(LEP_AGC_MODULE_BASE + 0x0044)
#define LEP_CID_AGC_CALC_ENABLE_STATE           (uint16_t)(LEP_AGC_MODULE_BASE + 0x0048)

enum LEP_AGC_POLICY {
    LEP_AGC_LINEAR = 0,
    LEP_AGC_HEQ,
};

struct LEP_AGC_HISTOGRAM_ROI {
    uint16_t startCol;
    uint16_t startRow;
    uint16_t endCol;
    uint16_t endRow;
};

struct LEP_AGC_HISTOGRAM_STATISTICS {
    uint16_t minIntensity;
    uint16_t maxIntensity;
    uint16_t meanIntensity;
    uint16_t numPixels; // def: 4800
};

enum LEP_AGC_HEQ_SCALE_FACTOR {
    LEP_AGC_SCALE_TO_8_BITS = 0,
    LEP_AGC_SCALE_TO_14_BITS
};

// SYS

#define LEP_SYS_MODULE_BASE                     (uint16_t)0x0200
#define LEP_CID_SYS_PING                        (uint16_t)(LEP_SYS_MODULE_BASE + 0x0000)
#define LEP_CID_SYS_CAM_STATUS                  (uint16_t)(LEP_SYS_MODULE_BASE + 0x0004)
#define LEP_CID_SYS_FLIR_SERIAL_NUMBER          (uint16_t)(LEP_SYS_MODULE_BASE + 0x0008)
#define LEP_CID_SYS_CAM_UPTIME                  (uint16_t)(LEP_SYS_MODULE_BASE + 0x000C)
#define LEP_CID_SYS_AUX_TEMPERATURE_KELVIN      (uint16_t)(LEP_SYS_MODULE_BASE + 0x0010)
#define LEP_CID_SYS_FPA_TEMPERATURE_KELVIN      (uint16_t)(LEP_SYS_MODULE_BASE + 0x0014)
#define LEP_CID_SYS_TELEMETRY_ENABLE_STATE      (uint16_t)(LEP_SYS_MODULE_BASE + 0x0018)
#define LEP_CID_SYS_TELEMETRY_LOCATION          (uint16_t)(LEP_SYS_MODULE_BASE + 0x001C)
#define LEP_CID_SYS_EXECTUE_FRAME_AVERAGE       (uint16_t)(LEP_SYS_MODULE_BASE + 0x0020)
#define LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE       (uint16_t)(LEP_SYS_MODULE_BASE + 0x0024)
#define LEP_CID_SYS_CUST_SERIAL_NUMBER          (uint16_t)(LEP_SYS_MODULE_BASE + 0x0028)
#define LEP_CID_SYS_SCENE_STATISTICS            (uint16_t)(LEP_SYS_MODULE_BASE + 0x002C)
#define LEP_CID_SYS_SCENE_ROI                   (uint16_t)(LEP_SYS_MODULE_BASE + 0x0030)
#define LEP_CID_SYS_THERMAL_SHUTDOWN_COUNT      (uint16_t)(LEP_SYS_MODULE_BASE + 0x0034)
#define LEP_CID_SYS_SHUTTER_POSITION            (uint16_t)(LEP_SYS_MODULE_BASE + 0x0038)
#define LEP_CID_SYS_FFC_SHUTTER_MODE            (uint16_t)(LEP_SYS_MODULE_BASE + 0x003C)
#define LEP_CID_SYS_RUN_FFC                     (uint16_t)(LEP_SYS_MODULE_BASE + 0x0042)
#define LEP_CID_SYS_FFC_STATUS                  (uint16_t)(LEP_SYS_MODULE_BASE + 0x0044)

enum LEP_SYS_CAM_STATUS_STATES {
    LEP_SYSTEM_READY = 0,
    LEP_SYSTEM_INITIALIZING,
    LEP_SYSTEM_IN_LOW_POWER_MODE,
    LEP_SYSTEM_GOING_INTO_STANDBY,
    LEP_SYSTEM_FLAT_FIELD_IN_PROCESS
};

struct LEP_SYS_CAM_STATUS {
    uint32_t camStatus;                         // LEP_SYS_CAM_STATUS_STATES
    uint16_t commandCount;
    uint16_t reserved;
};

enum LEP_SYS_TELEMETRY_LOCATION {
    LEP_TELEMETRY_LOCATION_HEADER = 0,
    LEP_TELEMETRY_LOCATION_FOOTER
};

enum LEP_SYS_FRAME_AVERAGE {
    LEP_SYS_FA_DIV_1 = 0,
    LEP_SYS_FA_DIV_2,
    LEP_SYS_FA_DIV_4,
    LEP_SYS_FA_DIV_8,
    LEP_SYS_FA_DIV_16,
    LEP_SYS_FA_DIV_32,
    LEP_SYS_FA_DIV_64,
    LEP_SYS_FA_DIV_128
};

struct LEP_SYS_SCENE_STATISTICS {
    uint16_t meanIntensity;
    uint16_t maxIntensity;
    uint16_t minIntensity;
    uint16_t numPixels;
};

struct LEP_SYS_SCENE_ROI {
    uint16_t startCol;
    uint16_t startRow;
    uint16_t endCol;
    uint16_t endRow;
};

enum LEP_SYS_SHUTTER_POSITION {
    LEP_SYS_SHUTTER_POSITION_UNKNOWN = -1,
    LEP_SYS_SHUTTER_POSITION_IDLE = 0,
    LEP_SYS_SHUTTER_POSITION_OPEN,
    LEP_SYS_SHUTTER_POSITION_CLOSED,
    LEP_SYS_SHUTTER_POSITION_BRAKE_ON
};

enum LEP_SYS_FFC_SHUTTER_MODE_STATE {
    LEP_SYS_FFC_SHUTTER_MODE_MANUAL = 0,
    LEP_SYS_FFC_SHUTTER_MODE_AUTO,
    LEP_SYS_FFC_SHUTTER_MODE_EXTERNAL
};

enum LEP_SYS_SHUTTER_TEMP_LOCKOUT_STATE {
    LEP_SYS_SHUTTER_LOCKOUT_INACTIVE = 0,
    LEP_SYS_SHUTTER_LOCKOUT_HIGH,
    LEP_SYS_SHUTTER_LOCKOUT_LOW
};

struct LEP_SYS_FFC_SHUTTER_MODE {
    uint32_t shutterMode;                       // LEP_SYS_FFC_SHUTTER_MODE_STATE def:LEP_SYS_FFC_SHUTTER_MODE_EXTERNAL
    uint32_t tempLockoutState;                  // LEP_SYS_SHUTTER_TEMP_LOCKOUT_STATE def:LEP_SYS_SHUTTER_LOCKOUT_INACTIVE
    uint32_t videoFreezeDuringFFC;              // bool def:enabled
    uint32_t ffcDesired;                        // bool def:disabled
    uint32_t elapsedTimeSinceLastFFC;           // (ms)
    uint32_t desiredFFCPeriod;                  // def:300000 (ms)
    uint32_t explicitCmdToOpen;                 // bool def:disabled
    uint16_t desiredFFCTempDelta;               // def:300 (kelvins*100)
    uint16_t imminentDelay;                     // def:52 (frame counts)
};

enum LEP_SYS_FFC_STATUS {
    LEP_SYS_FFC_STATUS_WRITE_ERROR = -2,
    LEP_SYS_FFC_STATUS_ERROR = -1,
    LEP_SYS_FFC_STATUS_READY = 0,
    LEP_SYS_FFC_STATUS_BUSY,
    LEP_SYS_FRAME_AVERAGE_COLLECTING_FRAMES
};

// VID

#define LEP_VID_MODULE_BASE                     (uint16_t)0x0300
#define LEP_CID_VID_POLARITY_SELECT             (uint16_t)(LEP_VID_MODULE_BASE + 0x0000)
#define LEP_CID_VID_LUT_SELECT                  (uint16_t)(LEP_VID_MODULE_BASE + 0x0004)
#define LEP_CID_VID_LUT_TRANSFER                (uint16_t)(LEP_VID_MODULE_BASE + 0x0008)
#define LEP_CID_VID_FOCUS_CALC_ENABLE           (uint16_t)(LEP_VID_MODULE_BASE + 0x000C)
#define LEP_CID_VID_FOCUS_ROI                   (uint16_t)(LEP_VID_MODULE_BASE + 0x0010)
#define LEP_CID_VID_FOCUS_THRESHOLD             (uint16_t)(LEP_VID_MODULE_BASE + 0x0014)
#define LEP_CID_VID_FOCUS_METRIC                (uint16_t)(LEP_VID_MODULE_BASE + 0x0018)
#define LEP_CID_VID_SBNUC_ENABLE                (uint16_t)(LEP_VID_MODULE_BASE + 0x001C)
#define LEP_CID_VID_GAMMA_SELECT                (uint16_t)(LEP_VID_MODULE_BASE + 0x0020)
#define LEP_CID_VID_FREEZE_ENABLE               (uint16_t)(LEP_VID_MODULE_BASE + 0x0024)
#define LEP_CID_VID_OUTPUT_FORMAT               (uint16_t)(LEP_VID_MODULE_BASE + 0x0030)

enum LEP_VID_POLARITY {
    LEP_VID_WHITE_HOT = 0,
    LEP_VID_BLACK_HOT
};

enum LEP_VID_PCOLOR_LUT {
    LEP_VID_WHEEL6_LUT = 0,
    LEP_VID_FUSION_LUT,
    LEP_VID_RAINBOW_LUT,
    LEP_VID_GLOBOW_LUT,
    LEP_VID_SEPIA_LUT,
    LEP_VID_COLOR_LUT,
    LEP_VID_ICE_FIRE_LUT,
    LEP_VID_RAIN_LUT,
    LEP_VID_USER_LUT,
};

struct LEP_VID_LUT_PIXEL {
    uint8_t reserved;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

struct LEP_VID_LUT_BUFFER {
    LEP_VID_LUT_PIXEL bin[256];
};

struct LEP_VID_FOCUS_ROI {
    uint16_t startCol;
    uint16_t startRow;
    uint16_t endCol;
    uint16_t endRow;
};

enum LEP_VID_VIDEO_OUTPUT_FORMAT {
     LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8 = 0,
     LEP_VID_VIDEO_OUTPUT_FORMAT_RAW10,
     LEP_VID_VIDEO_OUTPUT_FORMAT_RAW12,
     LEP_VID_VIDEO_OUTPUT_FORMAT_RGB888,
     LEP_VID_VIDEO_OUTPUT_FORMAT_RGB666,
     LEP_VID_VIDEO_OUTPUT_FORMAT_RGB565,
     LEP_VID_VIDEO_OUTPUT_FORMAT_YUV422_8BIT,
     LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14,
     LEP_VID_VIDEO_OUTPUT_FORMAT_YUV422_10BIT,
     LEP_VID_VIDEO_OUTPUT_FORMAT_USER_DEFINED,
     LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_2,
     LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_3,
     LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_4,
     LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_5,
     LEP_VID_VIDEO_OUTPUT_FORMAT_RAW8_6
};

// OEM

#define LEP_OEM_MODULE_BASE                     (uint16_t)0x0800
#define LEP_CID_OEM_POWER_MODE                  (uint16_t)(LEP_OEM_MODULE_BASE + 0x0000)
#define LEP_CID_OEM_FLIR_SERIAL_NUMBER          (uint16_t)(LEP_OEM_MODULE_BASE + 0x001C)
#define LEP_CID_OEM_SOFTWARE_REVISION           (uint16_t)(LEP_OEM_MODULE_BASE + 0x0020)
#define LEP_CID_OEM_VID_OUTPUT_ENABLE           (uint16_t)(LEP_OEM_MODULE_BASE + 0x0024)
#define LEP_CID_OEM_VID_OUTPUT_FORMAT           (uint16_t)(LEP_OEM_MODULE_BASE + 0x0028)
#define LEP_CID_OEM_CUST_PART_NUMBER            (uint16_t)(LEP_OEM_MODULE_BASE + 0x0038)
#define LEP_CID_OEM_OUTPUT_SOURCE_CONST         (uint16_t)(LEP_OEM_MODULE_BASE + 0x003C)
#define LEP_CID_OEM_CAMERA_REBOOT               (uint16_t)(LEP_OEM_MODULE_BASE + 0x0040)
#define LEP_CID_OEM_FFC_NORM_TARGET             (uint16_t)(LEP_OEM_MODULE_BASE + 0x0044)
#define LEP_CID_OEM_STATUS                      (uint16_t)(LEP_OEM_MODULE_BASE + 0x0048)
#define LEP_CID_OEM_FRAME_MEAN                  (uint16_t)(LEP_OEM_MODULE_BASE + 0x004C)
#define LEP_CID_OEM_GPIO_MODE                   (uint16_t)(LEP_OEM_MODULE_BASE + 0x0054)
#define LEP_CID_OEM_VSYNC_DELAY                 (uint16_t)(LEP_OEM_MODULE_BASE + 0x0058)
#define LEP_CID_OEM_USER_PARAMS_STATE           (uint16_t)(LEP_OEM_MODULE_BASE + 0x005C)
#define LEP_CID_OEM_USER_PARAMS_RESTORE         (uint16_t)(LEP_OEM_MODULE_BASE + 0x0060)
#define LEP_CID_OEM_SHUTTER_PROFILE             (uint16_t)(LEP_OEM_MODULE_BASE + 0x0064)
#define LEP_CID_OEM_THERM_SHUTDOWN_ENABLE       (uint16_t)(LEP_OEM_MODULE_BASE + 0x0068)
#define LEP_CID_OEM_BAD_PX_REPLACE_ENABLE       (uint16_t)(LEP_OEM_MODULE_BASE + 0x006C)
#define LEP_CID_OEM_TEMPORAL_FILTER_ENABLE      (uint16_t)(LEP_OEM_MODULE_BASE + 0x0070)
#define LEP_CID_OEM_COL_NOISE_FILTER_ENABLE     (uint16_t)(LEP_OEM_MODULE_BASE + 0x0074)
#define LEP_CID_OEM_PX_NOISE_FILTER_ENABLE      (uint16_t)(LEP_OEM_MODULE_BASE + 0x0078)
#define LEP_CID_OEM_FFC_NORMALIZATION           (uint16_t)(LEP_OEM_MODULE_BASE + 0x007C)

enum LEP_OEM_VIDEO_OUTPUT_SOURCE {
    LEP_VIDEO_OUTPUT_SOURCE_RAW = 0,            // Before video processing.
    LEP_VIDEO_OUTPUT_SOURCE_COOKED,             // Post video processing -NORMAL MODE
    LEP_VIDEO_OUTPUT_SOURCE_RAMP,               // Software Ramp pattern -increase in X, Y
    LEP_VIDEO_OUTPUT_SOURCE_CONSTANT,           // Software Constant value pattern
    LEP_VIDEO_OUTPUT_SOURCE_RAMP_H,             // Software Ramp pattern -increase in X only
    LEP_VIDEO_OUTPUT_SOURCE_RAMP_V,             // Software Ramp pattern -increase in Y only
    LEP_VIDEO_OUTPUT_SOURCE_RAMP_CUSTOM,        // Software Ramp pattern -uses custom settings
    // Additions to support frame averaging, freeze frame, and data buffers
    LEP_VIDEO_OUTPUT_SOURCE_FRAME_CAPTURE,      // Average, Capture frame
    LEP_VIDEO_OUTPUT_SOURCE_FRAME_FREEZE,       // Freeze-Frame Buffer
    // Reserved buffers
    LEP_VIDEO_OUTPUT_SOURCE_FRAME_0,            // Reserved DATA Buffer
    LEP_VIDEO_OUTPUT_SOURCE_FRAME_1,            // Reserved DATA Buffer
    LEP_VIDEO_OUTPUT_SOURCE_FRAME_2,            // Reserved DATA Buffer
    LEP_VIDEO_OUTPUT_SOURCE_FRAME_3,            // Reserved DATA Buffer
    LEP_VIDEO_OUTPUT_SOURCE_FRAME_4             // Reserved DATA Buffer
};

enum LEP_OEM_STATUS {
    LEP_OEM_STATUS_OTP_WRITE_ERROR = -2,
    LEP_OEM_STATUS_ERROR = -1,
    LEP_OEM_STATUS_READY = 0,
    LEP_OEM_STATUS_BUSY,
    LEP_OEM_FRAME_AVERAGE_COLLECTING_FRAMES
};

enum LEP_OEM_GPIO_MODE {
    LEP_OEM_GPIO_MODE_GPIO = 0,
    LEP_OEM_GPIO_MODE_I2C_MASTER = 1,
    LEP_OEM_GPIO_MODE_SPI_MASTER_VLB_DATA = 2,
    LEP_OEM_GPIO_MODE_SPIO_MASTER_REG_DATA = 3,
    LEP_OEM_GPIO_MODE_SPI_SLAVE_VLB_DATA = 4,
    LEP_OEM_GPIO_MODE_VSYNC = 5
};

enum LEP_OEM_VSYNC_DELAY {
    LEP_OEM_VSYNC_DELAY_MINUS_3 = -3,
    LEP_OEM_VSYNC_DELAY_MINUS_2 = -2,
    LEP_OEM_VSYNC_DELAY_MINUS_1 = -1,
    LEP_OEM_VSYNC_DELAY_NONE = 0,
    LEP_OEM_VSYNC_DELAY_PLUS_1 = 1,
    LEP_OEM_VSYNC_DELAY_PLUS_2 = 2,
    LEP_OEM_VSYNC_DELAY_PLUS_3 = 3
};

enum LEP_OEM_USER_PARAMS_STATE {
    LEP_OEM_USER_PARAMS_STATE_NOT_WRITTEN = 0,
    LEP_OEM_USER_PARAMS_STATE_WRITTEN
};

struct LEP_OEM_SHUTTER_PROFILE {
    uint16_t closePeriodInFrames;               // in frame counts x1
    uint16_t openPeriodInFrames;                // in frame counts x1
};


// RAD

#define LEP_RAD_MODULE_BASE                     (uint16_t)0x0E00
#define LEP_CID_RAD_RFBO_PARAMS                 (uint16_t)(LEP_RAD_MODULE_BASE + 0x0004)
#define LEP_CID_RAD_RAD_ENABLE                  (uint16_t)(LEP_RAD_MODULE_BASE + 0x0010)
#define LEP_CID_RAD_TSHUTTER_MODE               (uint16_t)(LEP_RAD_MODULE_BASE + 0x0024)
#define LEP_CID_RAD_TSHUTTER_TEMP               (uint16_t)(LEP_RAD_MODULE_BASE + 0x0028)
#define LEP_CID_RAD_FFC_NORMALIZATION           (uint16_t)(LEP_RAD_MODULE_BASE + 0x002C)
#define LEP_CID_RAD_STATUS                      (uint16_t)(LEP_RAD_MODULE_BASE + 0x0030)
#define LEP_CID_RAD_FLUX_LINEAR_PARAMS          (uint16_t)(LEP_RAD_MODULE_BASE + 0x00BC)
#define LEP_CID_RAD_TLINEAR_ENABLE              (uint16_t)(LEP_RAD_MODULE_BASE + 0x00C0)
#define LEP_CID_RAD_TLINEAR_RESOLUTION          (uint16_t)(LEP_RAD_MODULE_BASE + 0x00C4)
#define LEP_CID_RAD_TLINEAR_AUTO_RESOLUTION     (uint16_t)(LEP_RAD_MODULE_BASE + 0x00C8)
#define LEP_CID_RAD_SPOTMETER_ROI               (uint16_t)(LEP_RAD_MODULE_BASE + 0x00CC)
#define LEP_CID_RAD_SPOTMETER_VALUES            (uint16_t)(LEP_RAD_MODULE_BASE + 0x00D0)
#define LEP_CID_RAD_LOW_GAIN_RBFO_PARAMS        (uint16_t)(LEP_RAD_MODULE_BASE + 0x00D8)

struct LEP_RBFO {
    uint32_t RBFO_R;                            // value is not scaled, min:10000 max:1000000 def:395653 scale:1 (norm), def:64155 (low gain)
    uint32_t RBFO_B;                            // value is scaled by X << n, min:1200000 max:1700000 def:1428000 scale:1000 (norm), def:1428000 (low gain)
    uint32_t RBFO_F;                            // min:500 max:3000 def:1000 scale:1000
    int32_t RBFO_O;                             // min:-16384000 max:16383000 def:156000 scale:1000 (norm), def:728000 (low gain)
};

enum LEP_RAD_TS_MODE {
    LEP_RAD_TS_USER_MODE = 0,
    LEP_RAD_TS_CAL_MODE,
    LEP_RAD_TS_FIXED_MODE
};

enum LEP_RAD_STATUS {
    LEP_RAD_STATUS_ERROR = -1,
    LEP_RAD_STATUS_READY = 0,
    LEP_RAD_STATUS_BUSY,
    LEP_RAD_FRAME_AVERAGE_COLLECTING_FRAMES
};

struct LEP_RAD_FLUX_LINEAR_PARAMS {
    uint16_t sceneEmissivity;                   // Scene emissivity, min:82 max:8192 def:8192 units:% scale:8192/100 (8192=100%)
    uint16_t TBkgK;                             // Background temperature, value in Kelvin 100x, min:0 max:65535 def:30000 units:K100 scale:100 (29515 = 295.15K)
    uint16_t tauWindow;                         // Window tau, min:82 max:8192 def:8192 units:% scale:8192/100 (8192=100%)
    uint16_t TWindowK;                          // Window temperature, value in Kelvin 100x, min:0 max:65535 def:30000 units:K100 scale:100 (29515 = 295.15K)
    uint16_t tauAtm;                            // Atmosphere tau, min:82 max:8192 def:8192 units:% scale:8192/100 (8192=100%)
    uint16_t TAtmK;                             // Atmosphere temperature, value in Kelvin 100x, min:0 max:65535 def:30000 units:K100 scale:100 (29515 = 295.15K)
    uint16_t tauReflWindow;                     // Reflection window tau, min:0 max:8192-tauWindow def:0 units:% scale:8192/100 (8192=100%)
    uint16_t TReflK;                            // Reflection window temperature, value in Kelvin 100x, min:0 max:65535 def:30000 units:K100 scale:100 (29515 = 295.15K)
};

enum LEP_RAD_TLINEAR_RESOLUTION {
    LEP_RAD_RESOLUTION_0_1 = 0,                 // TLinear 0.1 resolution, min pixel:0 max pixel:65535 units:K100 scale:100 (65535 = 655.35K)
    LEP_RAD_RESOLUTION_0_01                     // TLinear 0.01 resolution, min pixel:0 max pixel:65535 units:K100 scale:100 (65535 = 655.35K)
};

struct LEP_RAD_ROI {
    uint16_t startRow;                          // Start row, min:0 max:endRow-1 def:29 (Lepton v1-v2.5), def:59 (Lepton v3+)
    uint16_t startCol;                          // Start column, min:0 max:endCol-1 def:39 (Lepton v1-v2.5), def:79 (Lepton v3+)
    uint16_t endRow;                            // End row, min:startRow+1 max:59 def:30 (Lepton v1-v2.5), max:119 def:60 (Lepton v3+)
    uint16_t endCol;                            // End column, min:startCol+1 max:79 def:40 (Lepton v1-v2.5), max:159 def:80 (Lepton v3+)
};

struct LEP_RAD_SPOTMETER_VALUES {
    uint16_t radSpotmeterValue;                 // Value (mean?), min:0 max:65535 units:{K10,K100} scale:{10,100} (based on TLinear resolution)
    uint16_t radSpotmeterMaxValue;              // Min value, min:0 max:65535 units:{K10,K100} scale:{10,100} (based on TLinear resolution)
    uint16_t radSpotmeterMinValue;              // Max value, min:0 max:65535 units:{K10,K100} scale:{10,100} (based on TLinear resolution)
    uint16_t radSpotmeterPopulation;            // Population count, min:0 max:4800 units:pixels scale:1 (Lepton v1-v2.5), max:19200 (Lepton v3+)
};

// Result

enum LEP_RESULT {
    LEP_OK = 0,                                 // Camera ok
    LEP_COMM_OK = LEP_OK,                       // Camera comm ok (same as LEP_OK)

    LEP_ERROR = -1,                             // Camera general error
    LEP_NOT_READY = -2,                         // Camera not ready error
    LEP_RANGE_ERROR = -3,                       // Camera range error
    LEP_CHECKSUM_ERROR = -4,                    // Camera checksum error
    LEP_BAD_ARG_POINTER_ERROR = -5,             // Camera Bad argument error
    LEP_DATA_SIZE_ERROR = -6,                   // Camera byte count error
    LEP_UNDEFINED_FUNCTION_ERROR = -7,          // Camera undefined function error
    LEP_FUNCTION_NOT_SUPPORTED = -8,            // Camera function not yet supported error

    // OTP access errors
    LEP_OTP_WRITE_ERROR = -15,                  // Camera OTP write error
    LEP_OTP_READ_ERROR = -16,                   // Double bit error detected (uncorrectible)
    LEP_OTP_NOT_PROGRAMMED_ERROR = -18,         // Flag read as non-zero

    // I2C Errors
    LEP_ERROR_I2C_BUS_NOT_READY = -20,          // I2C Bus Error - Bus Not Avaialble
    LEP_ERROR_I2C_BUFFER_OVERFLOW = -22,        // I2C Bus Error - Buffer Overflow
    LEP_ERROR_I2C_ARBITRATION_LOST = -23,       // I2C Bus Error - Bus Arbitration Lost
    LEP_ERROR_I2C_BUS_ERROR = -24,              // I2C Bus Error - General Bus Error
    LEP_ERROR_I2C_NACK_RECEIVED = -25,          // I2C Bus Error - NACK Received
    LEP_ERROR_I2C_FAIL = -26,                   // I2C Bus Error - General Failure

    // Processing Errors
    LEP_DIV_ZERO_ERROR = -80,                   // Attempted div by zero

    // Comm Errors
    LEP_COMM_PORT_NOT_OPEN = -101,              // Comm port not open
    LEP_COMM_INVALID_PORT_ERROR = -102,         // Comm port no such port error
    LEP_COMM_RANGE_ERROR = -103,                // Comm port range error
    LEP_ERROR_CREATING_COMM = -104,             // Error creating comm
    LEP_ERROR_STARTING_COMM = -105,             // Error starting comm
    LEP_ERROR_CLOSING_COMM = -106,              // Error closing comm
    LEP_COMM_CHECKSUM_ERROR = -107,             // Comm checksum error
    LEP_COMM_NO_DEV = -108,                     // No comm device
    LEP_TIMEOUT_ERROR = -109,                   // Comm timeout error
    LEP_COMM_ERROR_WRITING_COMM = -110,         // Error writing comm
    LEP_COMM_ERROR_READING_COMM = -111,         // Error reading comm
    LEP_COMM_COUNT_ERROR = -112,                // Comm byte count error

    // Other Errors
    LEP_OPERATION_CANCELED = -126,              // Camera operation canceled
    LEP_UNDEFINED_ERROR_CODE = -127             // Undefined error
};

#endif // /#ifndef LeptonFLiRIDDDefs_H
