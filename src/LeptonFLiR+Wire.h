/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / i2c Communications
*/
#ifndef LeptonFLiR_Wire_H
#define LeptonFLiR_Wire_H

#include "LeptonFLiR.h"

#ifndef LEPFLIR_IN_PROTECTED
#error "This file not meant to be directly included. Include LeptonFLiR.h instead."
#endif
// class LeptonFLiR {
// protected:

    bool waitCommandBegin(int timeout = 0);
    bool waitCommandFinish(int timeout = 0);

    uint16_t cmdCode(uint16_t cmdID, uint16_t cmdType);

    void sendCommand(uint16_t cmdCode);
    void sendCommand(uint16_t cmdCode, uint16_t value);
    void sendCommand(uint16_t cmdCode, uint32_t value);
    void sendCommand(uint16_t cmdCode, uint16_t *dataWords, int dataLength);

    void receiveCommand(uint16_t cmdCode, uint16_t *value);
    void receiveCommand(uint16_t cmdCode, uint32_t *value);
    void receiveCommand(uint16_t cmdCode, uint16_t *readWords, int maxLength);

    int writeCmdRegister(uint16_t cmdCode, uint16_t *dataWords, int dataLength);
    int readDataRegister(uint16_t *readWords, int maxLength);

    int writeRegister(uint16_t regAddress, uint16_t value);
    int readRegister(uint16_t regAddress, uint16_t *value);

#ifdef LEPFLIR_USE_SOFTWARE_I2C
    uint8_t _readBytes;
#endif
    void i2cWire_begin();
    void i2cWire_beginTransmission(uint8_t);
    uint8_t i2cWire_endTransmission(void);
    uint8_t i2cWire_requestFrom(uint8_t, uint8_t);
    size_t i2cWire_write(uint8_t);
    size_t i2cWire_write16(uint16_t);
    uint8_t i2cWire_read(void);
    uint16_t i2cWire_read16(void);

#endif // /ifndef LeptonFLiR_Wire_H
