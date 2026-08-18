/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR / SPI Communications
*/

#include "LeptonFLiR.h"

#ifdef SPI_HAS_TRANSFER_ASYNC
bool LeptonFLiR::SPI_transfer16DMA(uint16_t *buffer, int count, bool store) {
    // A Lepton packet is at most 244 bytes. Keep the DMA receive buffer local
    // so its lifetime covers the entire asynchronous transfer.
    uint8_t rxBytes[244];

    while (count > 0) {
        const int words = min(count, 122);
        const size_t bytes = (size_t)words * 2;
        EventResponder event;

        if (!SPI.transfer(NULL, rxBytes, bytes, event))
            return false;

        while (!event)
            yield();

        if (store) {
            for (int i = 0; i < words; ++i)
                *buffer++ = ((uint16_t)rxBytes[i * 2] << 8) | rxBytes[i * 2 + 1];
        }

        count -= words;
    }

    return true;
}
#endif

void LeptonFLiR::SPI_transfer16(uint16_t *buffer, int count) {
#ifdef SPI_HAS_TRANSFER_ASYNC
    if (_spiDMAEnabled) {
        if (SPI_transfer16DMA(buffer, count, true))
            return;
        _spiDMAEnabled = false;
    }
#endif

#if defined(ARDUINO_ARCH_SAM)
    while (count-- > 0)
        *buffer++ = SPI.transfer16(0x0000, count ? SPI_CONTINUE : SPI_LAST);
#else
    while (count-- > 0)
        *buffer++ = SPI.transfer16(0x0000);
#endif
}

void LeptonFLiR::SPI_ignore16(int count) {
#ifdef SPI_HAS_TRANSFER_ASYNC
    if (_spiDMAEnabled) {
        if (SPI_transfer16DMA(NULL, count, false))
            return;
        _spiDMAEnabled = false;
    }
#endif

#if defined(ARDUINO_ARCH_SAM)
    while (count-- > 0)
        SPI.transfer16(0x0000, count ? SPI_CONTINUE : SPI_LAST);
#else
    while (count-- > 0)
        SPI.transfer16(0x0000);
#endif
}
