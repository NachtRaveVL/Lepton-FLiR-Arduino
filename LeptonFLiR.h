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

    Lepton-FLiR-Arduino - Version 0.1
*/

#ifndef LeptonFLiR_H
#define LeptonFLiR_H

#if !(defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD))
#error "This library can only be used with SAM/SAMD architectures (Due, Zero, etc.)."
#endif

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <Wire.h>
#include <SPI.h>

#include "LeptonFLiRDefs.h"

class LeptonFLiR
{
public:
    // May use a different Wire instance than Wire.
    LeptonFLiR(TwoWire& wire = Wire, byte csEnablePin = 53);

    void init();

private:
    TwoWire *_wire;         // Wire class instance to use
    byte _csEnablePin;
};

#endif
