/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016 NachtRaveVL      <nachtravevl@gmail.com>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    This permission notice shall be included in all copies or
    substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.

    Lepton-FLiR-Arduino - Version 0.5
*/

#ifndef LeptonFLiRDefs_H
#define LeptonFLiRDefs_H

#define LEP_SPI_FRAME_SIZE                      164 // 1B ID + 1B R# + 2B CRC + 160B for 80x1 14bpp/8bppAGC thermal image data

#define LEP_I2C_DEVICE_ADDRESS                  (uint8_t)0x2A

#define LEP_I2C_COMMAND_MODULE_ID_BIT_MASK      (uint16_t)0x0F00
#define LEP_I2C_COMMAND_ID_BIT_MASK             (uint16_t)0x00FC
#define LEP_I2C_COMMAND_TYPE_BIT_MASK           (uint16_t)0x0003

#define LEP_I2C_COMMAND_TYPE_GET                (uint16_t)0x0000
#define LEP_I2C_COMMAND_TYPE_SET                (uint16_t)0x0001
#define LEP_I2C_COMMAND_TYPE_RUN                (uint16_t)0x0002

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

#define LEP_I2C_DATA_BUFFER_0_BASE_ADDR         (uint16_t)0xF800
#define LEP_I2C_DATA_BUFFER_0_LENGTH            (uint16_t)0x0400
#define LEP_I2C_DATA_BUFFER_0                   (uint16_t)(LEP_I2C_DATA_BUFFER_0_BASE_ADDR)
#define LEP_I2C_DATA_BUFFER_0_END               (uint16_t)(LEP_I2C_DATA_BUFFER_0_BASE_ADDR + 0x03FF)
#define LEP_I2C_DATA_BUFFER_1_BASE_ADDR         (uint16_t)0xFC00
#define LEP_I2C_DATA_BUFFER_1_LENGTH            (uint16_t)0x0400
#define LEP_I2C_DATA_BUFFER_1                   (uint16_t)(LEP_I2C_DATA_BUFFER_1_BASE_ADDR)
#define LEP_I2C_DATA_BUFFER_1_END               (uint16_t)(LEP_I2C_DATA_BUFFER_1_BASE_ADDR + 0x03FF)

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
#define LEP_CID_SYS_FFC_SHUTTER_MODE_OBJ        (uint16_t)(LEP_SYS_MODULE_BASE + 0x003C)
#define FLR_CID_SYS_RUN_FFC                     (uint16_t)(LEP_SYS_MODULE_BASE + 0x0042)
#define LEP_CID_SYS_FFC_STATUS                  (uint16_t)(LEP_SYS_MODULE_BASE + 0x0044)

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

#define LEP_OEM_MODULE_BASE                     (uint16_t)0x0800
#define LEP_CID_OEM_CHIP_MASK_REVISION          (uint16_t)(LEP_OEM_MODULE_BASE + 0x0014)
#define LEP_CID_OEM_PART_NUMBER                 (uint16_t)(LEP_OEM_MODULE_BASE + 0x001C)
#define LEP_CID_OEM_CAM_SOFTWARE_REVISION       (uint16_t)(LEP_OEM_MODULE_BASE + 0x0020)

typedef enum {
    LEP_TELEMETRY_LOCATION_HEADER = 0,
    LEP_TELEMETRY_LOCATION_FOOTER
} LEP_SYS_TELEMETRY_LOCATION;

typedef enum {
    LEP_OK = 0,     /* Camera ok */
    LEP_COMM_OK = LEP_OK, /* Camera comm ok (same as LEP_OK) */

    LEP_ERROR = -1,    /* Camera general error */
    LEP_NOT_READY = -2,    /* Camera not ready error */
    LEP_RANGE_ERROR = -3,    /* Camera range error */
    LEP_CHECKSUM_ERROR = -4,    /* Camera checksum error */
    LEP_BAD_ARG_POINTER_ERROR = -5,    /* Camera Bad argument  error */
    LEP_DATA_SIZE_ERROR = -6,    /* Camera byte count error */
    LEP_UNDEFINED_FUNCTION_ERROR = -7,    /* Camera undefined function error */
    LEP_FUNCTION_NOT_SUPPORTED = -8,    /* Camera function not yet supported error */

    /* OTP access errors */
    LEP_OTP_WRITE_ERROR = -15,   /*!< Camera OTP write error */
    LEP_OTP_READ_ERROR = -16,   /* double bit error detected (uncorrectible) */

    LEP_OTP_NOT_PROGRAMMED_ERROR = -18,   /* Flag read as non-zero */

    /* I2C Errors */
    LEP_ERROR_I2C_BUS_NOT_READY = -20,   /* I2C Bus Error - Bus Not Avaialble */
    LEP_ERROR_I2C_BUFFER_OVERFLOW = -22,   /* I2C Bus Error - Buffer Overflow */
    LEP_ERROR_I2C_ARBITRATION_LOST = -23,   /* I2C Bus Error - Bus Arbitration Lost */
    LEP_ERROR_I2C_BUS_ERROR = -24,   /* I2C Bus Error - General Bus Error */
    LEP_ERROR_I2C_NACK_RECEIVED = -25,   /* I2C Bus Error - NACK Received */
    LEP_ERROR_I2C_FAIL = -26,   /* I2C Bus Error - General Failure */

    /* Processing Errors */
    LEP_DIV_ZERO_ERROR = -80,   /* Attempted div by zero */

    /* Comm Errors */
    LEP_COMM_PORT_NOT_OPEN = -101,  /* Comm port not open */
    LEP_COMM_INVALID_PORT_ERROR = -102,  /* Comm port no such port error */
    LEP_COMM_RANGE_ERROR = -103,  /* Comm port range error */
    LEP_ERROR_CREATING_COMM = -104,  /* Error creating comm */
    LEP_ERROR_STARTING_COMM = -105,  /* Error starting comm */
    LEP_ERROR_CLOSING_COMM = -106,  /* Error closing comm */
    LEP_COMM_CHECKSUM_ERROR = -107,  /* Comm checksum error */
    LEP_COMM_NO_DEV = -108,  /* No comm device */
    LEP_TIMEOUT_ERROR = -109,  /* Comm timeout error */
    LEP_COMM_ERROR_WRITING_COMM = -110,  /* Error writing comm */
    LEP_COMM_ERROR_READING_COMM = -111,  /* Error reading comm */
    LEP_COMM_COUNT_ERROR = -112,  /* Comm byte count error */

    /* Other Errors */
    LEP_OPERATION_CANCELED = -126,  /* Camera operation canceled */
    LEP_UNDEFINED_ERROR_CODE = -127   /* Undefined error */
} LEP_RESULT;

#endif
