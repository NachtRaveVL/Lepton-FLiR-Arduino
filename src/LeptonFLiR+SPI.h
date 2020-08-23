/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / SPI Communications
*/
#ifndef LeptonFLiR_SPI_H
#define LeptonFLiR_SPI_H

#include "LeptonFLiR.h"

#ifndef LEPFLIR_IN_PROTECTED
#error "This file not meant to be directly included. Include LeptonFLiR.h instead."
#endif
// class LeptonFLiR {
// protected:

    void SPI_begin();
    static void SPI_transfer16(uint16_t *buffer, int count);
    static void SPI_ignore16(int count);

#endif // /ifndef LeptonFLiR_SPI_H
