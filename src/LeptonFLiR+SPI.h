/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / SPI Communications
*/
#ifndef LeptonFLiR_SPI_H
#define LeptonFLiR_SPI_H

#include "LeptonFLiR.h"

#ifndef LEPFLIR_IN_PROTECTED
#error "This file not meant to be directly included. Include LeptonFLiR.h instead."
#endif
//class LeptonFLiR {
//protected:

    void SPI_transfer16(uint16_t *buffer, int count);
    void SPI_ignore16(int count);
#ifdef SPI_HAS_TRANSFER_ASYNC
    bool SPI_transfer16DMA(uint16_t *buffer, int count, bool store);
#endif

#endif // /ifndef LeptonFLiR_SPI_H
