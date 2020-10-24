/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / Utils Module
*/
#ifndef LeptonFLiR_Utils_H
#define LeptonFLiR_Utils_H

#include "LeptonFLiR.h"

#ifndef LEPFLIR_IN_PUBLIC
#error "This file not meant to be directly included. Include LeptonFLiR.h instead."
#endif
//class LeptonFLiR {
//public:

    // Module represents temperatures as kelvin x 100 (in integer format). These methods
    // convert to and from the selected temperature mode.
    float kelvin100ToTemperature(uint16_t kelvin100);
    uint16_t temperatureToKelvin100(float temperature);
    const char *getTemperatureSymbol();

    byte getLastI2CError();
    LEP_RESULT getLastLepResult();

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    float getCameraVersion();
    int getWireInterfaceNumber();
    void printModuleInfo();
    void checkForErrors();
#endif

    static void wordsToHexString(uint16_t *dataWords, int dataLength, char *buffer, int maxLength);

#endif // /ifndef LeptonFLiR_Utils_H
