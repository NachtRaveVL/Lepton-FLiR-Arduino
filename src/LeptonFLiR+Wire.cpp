/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / i2c Communications
*/

#include "LeptonFLiR.h"

#ifdef LEPFLIR_USE_SOFTWARE_I2C
boolean __attribute__((noinline)) i2c_init(void);
bool __attribute__((noinline)) i2c_start(uint8_t addr);
void __attribute__((noinline)) LEPFLIR_i2c_stop(void) asm("ass_i2c_stop");
bool __attribute__((noinline)) LEPFLIR_i2c_write(uint8_t value) asm("ass_i2c_write");
uint8_t __attribute__((noinline)) i2c_read(bool last);
#endif

bool LeptonFLiR::waitCommandBegin(int timeout) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("    LeptonFLiR::waitCommandBegin"));
#endif

    _lastLepResult = 0;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("  "));
#endif
    uint16_t status;
    if (readRegister(LEP_I2C_STATUS_REG, &status))
        return false;

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK))
        return true;

    unsigned long endTime = millis() + (unsigned long)timeout;

    while ((status & LEP_I2C_STATUS_BUSY_BIT_MASK) && (timeout <= 0 || millis() < endTime)) {
#ifdef LEPFLIR_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.print(F("  "));
#endif

        if (readRegister(LEP_I2C_STATUS_REG, &status))
            return false;
    }

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK))
        return true;
    else {
        _lastLepResult = LEP_TIMEOUT_ERROR;
        return false;
    }
}

bool LeptonFLiR::waitCommandFinish(int timeout) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println(F("    LeptonFLiR::waitCommandFinish"));
#endif

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("  "));
#endif
    uint16_t status;
    if (readRegister(LEP_I2C_STATUS_REG, &status))
        return false;

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK)) {
        _lastLepResult = (byte)((status & LEP_I2C_STATUS_ERROR_CODE_BIT_MASK) >> LEP_I2C_STATUS_ERROR_CODE_BIT_SHIFT);
        return true;
    }

    unsigned long endTime = millis() + (unsigned long)timeout;

    while ((status & LEP_I2C_STATUS_BUSY_BIT_MASK) && (timeout <= 0 || millis() < endTime)) {
#ifdef LEPFLIR_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.print(F("  "));
#endif

        if (readRegister(LEP_I2C_STATUS_REG, &status))
            return false;
    }

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK)) {
        _lastLepResult = (byte)((status & LEP_I2C_STATUS_ERROR_CODE_BIT_MASK) >> LEP_I2C_STATUS_ERROR_CODE_BIT_SHIFT);
        return true;
    }
    else {
        _lastLepResult = LEP_TIMEOUT_ERROR;
        return false;
    }
}

uint16_t LeptonFLiR::cmdCode(uint16_t cmdID, uint16_t cmdType) {
    return (cmdID & LEP_I2C_COMMAND_MODULE_ID_BIT_MASK) |
           (cmdID & LEP_I2C_COMMAND_ID_BIT_MASK) |
           (cmdType & LEP_I2C_COMMAND_TYPE_BIT_MASK) |
           ((cmdID & LEP_I2C_COMMAND_MODULE_ID_BIT_MASK) == LEP_OEM_MODULE_BASE ||
            (cmdID & LEP_I2C_COMMAND_MODULE_ID_BIT_MASK) == LEP_RAD_MODULE_BASE ? LEP_I2C_COMMAND_PROT_BIT : (uint16_t)0);
}

void LeptonFLiR::sendCommand(uint16_t cmdCode) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("  LeptonFLiR::sendCommand cmdCode: 0x"));
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, NULL, 0) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint16_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("  LeptonFLiR::sendCommand cmdCode: 0x"));
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, &value, 1) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint32_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("  LeptonFLiR::sendCommand cmdCode: 0x"));
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, (uint16_t *)&value, 2) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint16_t *dataWords, int dataLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("  LeptonFLiR::sendCommand cmdCode: 0x"));
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, dataWords, dataLength) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::receiveCommand(uint16_t cmdCode, uint16_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("  LeptonFLiR::receiveCommand cmdCode: 0x"));
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_DATA_LENGTH_REG, 1) == 0 &&
            writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

            if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

                readDataRegister(value, 1);
            }
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::receiveCommand(uint16_t cmdCode, uint32_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("  LeptonFLiR::receiveCommand cmdCode: 0x"));
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_DATA_LENGTH_REG, 2) == 0 &&
            writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

            if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

                readDataRegister((uint16_t *)value, 2);
            }
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::receiveCommand(uint16_t cmdCode, uint16_t *readWords, int maxLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("  LeptonFLiR::receiveCommand cmdCode: 0x"));
    Serial.println(cmdCode, HEX);
#endif

    if (!readWords || maxLength <= 0) {
        _lastI2CError = 4;
        return;
    }

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_DATA_LENGTH_REG, (uint16_t)maxLength) == 0 &&
            writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

            if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

                readDataRegister(readWords, maxLength);
            }
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

int LeptonFLiR::writeCmdRegister(uint16_t cmdCode, uint16_t *dataWords, int dataLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("    LeptonFLiR::writeCmdRegister cmdCode: 0x"));
    Serial.print(cmdCode, HEX);
    Serial.print(F(", dataWords["));
    Serial.print(dataLength);
    Serial.print(F("]: "));
    for (int i = 0; i < dataLength; ++i) {
        Serial.print(i > 0 ? F("-0x") : F("0x"));
        Serial.print(dataWords[i], HEX);
    }
    Serial.println("");
#endif

    if (dataLength < 0 || (dataLength > 0 && !dataWords))
        return (_lastI2CError = 4);

    // DATA_LENGTH is expressed in 16-bit words. FLIR's CCI protocol requires it
    // to be written for every command, including zero-length RUN commands.
    if (writeRegister(LEP_I2C_DATA_LENGTH_REG, (uint16_t)dataLength))
        return _lastI2CError;

    if (dataLength > 0) {
        const int maxWordsPerTransfer = max(1, LEPFLIR_I2C_BUFFER_LENGTH / 2 - 1);
        uint16_t regAddress = dataLength <= 16 ? LEP_I2C_DATA_0_REG : LEP_I2C_DATA_BUFFER;
        int remaining = dataLength;

        while (remaining > 0) {
            const int writeLength = min(maxWordsPerTransfer, remaining);

            i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
            i2cWire_write16(regAddress);

            for (int i = 0; i < writeLength; ++i)
                i2cWire_write16(*dataWords++);

            if (i2cWire_endTransmission())
                return _lastI2CError;

            regAddress += (uint16_t)(writeLength * 2);
            remaining -= writeLength;
        }
    }

    return writeRegister(LEP_I2C_COMMAND_REG, cmdCode);
}

int LeptonFLiR::readDataRegister(uint16_t *readWords, int maxLength) {
    if (!readWords || maxLength <= 0)
        return (_lastI2CError = 4);

    const int maxWordsPerTransfer = max(1, LEPFLIR_I2C_BUFFER_LENGTH / 2);
    uint16_t regAddress = maxLength <= 16 ? LEP_I2C_DATA_0_REG : LEP_I2C_DATA_BUFFER;
    int remaining = maxLength;

    while (remaining > 0) {
        const int readLength = min(maxWordsPerTransfer, remaining);
        const int byteLength = readLength * 2;

        i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
        i2cWire_write16(regAddress);
        if (i2cWire_endTransmission())
            return _lastI2CError;

        int bytesRead = i2cWire_requestFrom(LEP_I2C_DEVICE_ADDRESS, (uint8_t)byteLength);
        if (bytesRead != byteLength) {
            while (bytesRead-- > 0)
                i2cWire_read();
            return (_lastI2CError = 4);
        }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        uint16_t *origReadWords = readWords;
#endif

        for (int i = 0; i < readLength; ++i)
            *readWords++ = i2cWire_read16();

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.print(F("      LeptonFLiR::readDataRegister readWords["));
        Serial.print(readLength);
        Serial.print(F("]: "));
        for (int i = 0; i < readLength; ++i) {
            Serial.print(i > 0 ? F("-0x") : F("0x"));
            Serial.print(origReadWords[i], HEX);
        }
        Serial.println("");
#endif

        regAddress += (uint16_t)(readLength * 2);
        remaining -= readLength;
    }

    return (_lastI2CError = 0);
}

int LeptonFLiR::writeRegister(uint16_t regAddress, uint16_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("    LeptonFLiR::writeRegister regAddress: 0x"));
    Serial.print(regAddress, HEX);
    Serial.print(F(", value: 0x"));
    Serial.println(value, HEX);
#endif

    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    i2cWire_write16(regAddress);
    i2cWire_write16(value);
    return i2cWire_endTransmission();
}

int LeptonFLiR::readRegister(uint16_t regAddress, uint16_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("    LeptonFLiR::readRegister regAddress: 0x"));
    Serial.println(regAddress, HEX);
#endif

    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    i2cWire_write16(regAddress);
    if (i2cWire_endTransmission())
        return _lastI2CError;

    int bytesRead = i2cWire_requestFrom(LEP_I2C_DEVICE_ADDRESS, 2);
    if (bytesRead != 2) {
        while (bytesRead-- > 0)
            i2cWire_read();
        return (_lastI2CError = 4);
    }

    *value = i2cWire_read16();

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print(F("      LeptonFLiR::readRegister retVal: 0x"));
    Serial.println(*value, HEX);
#endif

    return _lastI2CError;
}

void LeptonFLiR::i2cWire_beginTransmission(uint8_t addr) {
    _lastI2CError = 0;
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    _i2cWire->beginTransmission(addr);
#else
    i2c_start(addr);
#endif
}

uint8_t LeptonFLiR::i2cWire_endTransmission(void) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return (_lastI2CError = _i2cWire->endTransmission());
#else
    LEPFLIR_i2c_stop();
    return (_lastI2CError = 0);
#endif
}

uint8_t LeptonFLiR::i2cWire_requestFrom(uint8_t addr, uint8_t len) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cWire->requestFrom(addr, (size_t)len);
#else
    i2c_start(addr | 0x01);
    return (_readBytes = len);
#endif
}

size_t LeptonFLiR::i2cWire_write(uint8_t data) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cWire->write(data);
#else
    return (size_t)LEPFLIR_i2c_write(data);
#endif
}

size_t LeptonFLiR::i2cWire_write16(uint16_t data) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cWire->write(highByte(data)) + _i2cWire->write(lowByte(data));
#else
    return (size_t)LEPFLIR_i2c_write(highByte(data)) + (size_t)LEPFLIR_i2c_write(lowByte(data));
#endif
}

uint8_t LeptonFLiR::i2cWire_read(void) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return (uint8_t)(_i2cWire->read() & 0xFF);
#else
    if (_readBytes > 1) {
        _readBytes -= 1;
        return (uint8_t)(i2c_read(false) & 0xFF);
    }
    else {
        _readBytes = 0;
        return (uint8_t)(i2c_read(true) & 0xFF);
    }
#endif
}

uint16_t LeptonFLiR::i2cWire_read16(void) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return ((uint16_t)(_i2cWire->read() & 0xFF) << 8) | (uint16_t)(_i2cWire->read() & 0xFF);
#else
    if (_readBytes > 2) {
        _readBytes -= 2;
        return ((uint16_t)(i2c_read(false) & 0xFF) << 8) | (uint16_t)(i2c_read(false) & 0xFF);
    }
    else {
        _readBytes = 0;
        return ((uint16_t)(i2c_read(false) & 0xFF) << 8) | (uint16_t)(i2c_read(true) & 0xFF);
    }
#endif
}
