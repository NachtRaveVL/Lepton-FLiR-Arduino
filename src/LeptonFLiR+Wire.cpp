/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / i2c Communications
*/

#include "LeptonFLiR.h"

bool LeptonFLiR::_i2cBegan = false;

bool LeptonFLiR::waitCommandBegin(int timeout) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.println("    LeptonFLiR::waitCommandBegin");
#endif

    _lastLepResult = 0;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  ");
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
        Serial.print("  ");
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
    Serial.println("    LeptonFLiR::waitCommandFinish");
#endif

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  ");
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
        Serial.print("  ");
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
           (cmdID & LEP_I2C_COMMAND_MODULE_ID_BIT_MASK == LEP_OEM_MODULE_BASE ||
            cmdID & LEP_I2C_COMMAND_MODULE_ID_BIT_MASK == LEP_RAD_MODULE_BASE ? LEP_I2C_COMMAND_PROT_BIT : 0);
}

void LeptonFLiR::sendCommand(uint16_t cmdCode) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
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
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
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
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
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
    Serial.print("  LeptonFLiR::sendCommand cmdCode: 0x");
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
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

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
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

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
    Serial.print("  LeptonFLiR::receiveCommand cmdCode: 0x");
    Serial.println(cmdCode, HEX);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

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
    Serial.print("    LeptonFLiR::writeCmdRegister cmdCode: 0x");
    Serial.print(cmdCode, HEX);
    Serial.print(", dataWords[");
    Serial.print(dataLength);
    Serial.print("]: ");
    for (int i = 0; i < dataLength; ++i) {
        Serial.print(i > 0 ? "-0x" : "0x");
        Serial.print(dataWords[i], HEX);
    }
    Serial.println("");
#endif

    // In avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
    // how many words can be written at once. Therefore, we loop around until all words
    // have been written out into their registers.

    if (dataWords && dataLength) {
        i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
        i2cWire_write16(LEP_I2C_DATA_LENGTH_REG);
        i2cWire_write16(dataLength);
        if (i2cWire_endTransmission())
            return _lastI2CError;

        int maxLength = LEPFLIR_I2C_BUFFER_LENGTH / 2;
        int writeLength = min(maxLength, dataLength);
        uint16_t regAddress = dataLength <= 16 ? LEP_I2C_DATA_0_REG : LEP_I2C_DATA_BUFFER;

        while (dataLength > 0) {
            i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
            i2cWire_write16(regAddress);

            while (writeLength-- > 0)
                i2cWire_write16(*dataWords++);

            if (i2cWire_endTransmission())
                return _lastI2CError;

            regAddress += maxLength * 0x02;
            dataLength -= maxLength;
            writeLength = min(maxLength, dataLength);
        }
    }

    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    i2cWire_write16(LEP_I2C_COMMAND_REG);
    i2cWire_write16(cmdCode);
    return i2cWire_endTransmission();
}

int LeptonFLiR::readDataRegister(uint16_t *readWords, int maxLength) {
    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    i2cWire_write16(LEP_I2C_DATA_LENGTH_REG);
    if (i2cWire_endTransmission())
        return _lastI2CError;

    int bytesRead = i2cWire_requestFrom(LEP_I2C_DEVICE_ADDRESS, 2);
    if (bytesRead != 2) {
        while (bytesRead-- > 0)
            i2cWire_read();
        return (_lastI2CError = 4);
    }

    int readLength = i2cWire_read16();

    if (readLength == 0)
        return (_lastI2CError = 4);

    // In avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
    // how many channels can be written at once. Therefore, we loop around until all
    // channels have been written out into their registers. I2C_BUFFER_LENGTH is also
    // used in other architectures, all of which goes into LEPFLIR_I2C_BUFFER_LENGTH.

    bytesRead = i2cWire_requestFrom(LEP_I2C_DEVICE_ADDRESS, min(LEPFLIR_I2C_BUFFER_LENGTH, readLength));

    while (bytesRead > 0 && readLength > 0) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        int origWordsRead = bytesRead / 2;
        int origReadLength = readLength / 2;
        int origMaxLength = maxLength;
        uint16_t *origReadWords = readWords;
#endif

        while (bytesRead > 1 && readLength > 1 && maxLength > 0) {
            *readWords++ = i2cWire_read16();
            bytesRead -= 2; readLength -= 2; --maxLength;
        }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        Serial.print("      LeptonFLiR::readDataRegister readWords[");
        if (origWordsRead == origReadLength && origReadLength == origMaxLength) {
            Serial.print(origWordsRead);
        }
        else if (origWordsRead != origReadLength && origReadLength == origMaxLength) {
            Serial.print("r:");
            Serial.print(origWordsRead);
            Serial.print(",lm:");
            Serial.print(origReadLength);
        }
        else {
            Serial.print("r:");
            Serial.print(origWordsRead);
            Serial.print(",l:");
            Serial.print(origReadLength);
            Serial.print(",m:");
            Serial.print(origMaxLength);
        }
        Serial.print("]: ");
        for (int i = 0; i < origWordsRead; ++i) {
            Serial.print(i > 0 ? "-0x" : "0x");
            Serial.print(origReadWords[i], HEX);
        }
        Serial.println("");
#endif

        if (readLength > 0)
            bytesRead += i2cWire_requestFrom(LEP_I2C_DEVICE_ADDRESS, min(LEPFLIR_I2C_BUFFER_LENGTH, readLength));
    }

    while (bytesRead-- > 0)
        i2cWire_read();

    while (maxLength-- > 0)
        *readWords++ = 0;

    return (_lastI2CError = readLength ? 4 : 0);
}

int LeptonFLiR::writeRegister(uint16_t regAddress, uint16_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("    LeptonFLiR::writeRegister regAddress: 0x");
    Serial.print(regAddress, HEX);
    Serial.print(", value: 0x");
    Serial.println(value, HEX);
#endif

    i2cWire_beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    i2cWire_write16(regAddress);
    i2cWire_write16(value);
    return i2cWire_endTransmission();
}

int LeptonFLiR::readRegister(uint16_t regAddress, uint16_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    Serial.print("    LeptonFLiR::readRegister regAddress: 0x");
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
    Serial.print("      LeptonFLiR::readRegister retVal: 0x");
    Serial.println(*value, HEX);
#endif

    return _lastI2CError;
}

#ifdef LEPFLIR_USE_SOFTWARE_I2C
bool __attribute__((noinline)) i2c_init(void);
bool __attribute__((noinline)) i2c_start(uint8_t addr);
void __attribute__((noinline)) i2c_stop(void) asm("ass_i2c_stop");
bool __attribute__((noinline)) i2c_write(uint8_t value) asm("ass_i2c_write");
uint8_t __attribute__((noinline)) i2c_read(bool last);
#endif

void LeptonFLiR::i2cWire_begin() {
    if (LeptonFLiR::_i2cBegan) return;
    LeptonFLiR::_i2cBegan = true;
    _lastI2CError = 0;
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    _i2cWire->begin();
    _i2cWire->setClock(getI2CSpeed());
#else
    if (!i2c_init()) _lastI2CError = 4;
#endif
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
    i2c_stop();
    return (_lastI2CError = 0);
#endif
}

uint8_t LeptonFLiR::i2cWire_requestFrom(uint8_t addr, uint8_t len) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cWire->requestFrom(addr, len);
#else
    i2c_start(addr | 0x01);
    return (_readBytes = len);
#endif
}

size_t LeptonFLiR::i2cWire_write(uint8_t data) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cWire->write(data);
#else
    return (size_t)i2c_write(data);
#endif
}

size_t LeptonFLiR::i2cWire_write16(uint16_t data) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return _i2cWire->write(highByte(data)) + _i2cWire->write(lowByte(data));
#else
    return (size_t)i2c_write(highByte(data)) + (size_t)i2c_write(lowByte(data));
#endif
}

uint8_t LeptonFLiR::i2cWire_read(void) {
#ifndef LEPFLIR_USE_SOFTWARE_I2C
    return (uint8_t)(_i2cWire->read() & 0xFF);
#else
    if (_readBytes > 1) {
        _readByes -= 1;
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
        readBytes -= 2;
        return ((uint16_t)(i2c_read(false) & 0xFF) << 8) | (uint16_t)(i2c_read(false) & 0xFF);
    }
    else {
        _readBytes = 0;
        return ((uint16_t)(i2c_read(false) & 0xFF) << 8) | (uint16_t)(i2c_read(true) & 0xFF);
    }
#endif
}
