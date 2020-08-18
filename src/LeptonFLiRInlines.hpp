/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    LeptonFLiR Inlines
*/
#ifndef LeptonFLiRInlines_HPP
#define LeptonFLiRInlines_HPP

#include "LeptonFLiR.h"

static inline int roundUpVal16(int val) { return ((val + 15) & -16); }
static inline byte *roundUpPtr16(byte *ptr) { return ptr ? (byte *)(((uintptr_t)ptr + 15) & -16) : NULL; }
static inline byte *roundUpMalloc16(int size) { return (byte *)malloc((size_t)(size + 15)); }
static inline byte *roundUpRealloc16(byte *ptr, int size) { return (byte *)realloc((void *)ptr, (size_t)(size + 15)); }

static inline void delayTimeout(int timeout) {
#ifdef LEPFLIR_USE_SCHEDULER
    unsigned long endTime = millis() + (unsigned long)timeout;
    while (millis() < endTime)
        Scheduler.yield();
#else
    delay(timeout);
#endif
}

#endif // /#ifndef LeptonFLiRInlines_HPP
