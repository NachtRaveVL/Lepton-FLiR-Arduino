/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / SPI Communications
*/

#include "LeptonFLiR.h"

bool LeptonFLiR::_spiBegan = false;

void LeptonFLiR::SPI_begin() {
    if (LeptonFLiR::_spiBegan) return;
    LeptonFLiR::_spiBegan = true;
#ifdef __SAM3X8E__
    // Arduino Due has SPI library that manages the CS pin for us
    SPI.begin(getChipSelectPin())
#else
    SPI.begin();
#endif
}

void LeptonFLiR::SPI_transfer16(uint16_t *buffer, int count) {
#if defined(ARDUINO_ARCH_SAM)
    while (count-- > 0)
        *buffer++ = SPI.transfer16(0x0000, count ? SPI_CONTINUE : SPI_LAST);
#else
    while (count-- > 0)
        *buffer++ = SPI.transfer16(0x0000);
#endif
}

void LeptonFLiR::SPI_ignore16(int count) {
#if defined(ARDUINO_ARCH_SAM)
    while (count-- > 0)
        SPI.transfer16(0x0000, count ? SPI_CONTINUE : SPI_LAST);
#else
    while (count-- > 0)
        SPI.transfer16(0x0000);
#endif
}
