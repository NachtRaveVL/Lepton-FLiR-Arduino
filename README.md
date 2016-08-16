# Lepton-FLiR-Arduino
Arduino Library for the Lepton FLiR Thermal Camera Module.

**Lepton-FLiR-Arduino v0.2**

Library to control a Lepton FLiR (forward looking infrared) thermal camera module from an Arduino board (Due, Zero, etc. recommended).  
Licensed under the non-restrictive MIT license.

Created by NachtRaveVL, August 1st, 2016.

This library is currently under development.

## Library Setup

There are several defines inside of the library's header file that allows for more fine-tuned control.

```Arduino
// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
//#define LEPFLIR_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define to disable usage of the Scheduler library on SAM/SAMD architecures.
//#define LEPFLIR_DISABLE_SCHEDULER       1   // https://github.com/arduino-libraries/Scheduler

// Uncomment this define to enable 16-byte aligned memory allocations (may improve performance).
//#define LEPFLIR_ENABLE_ALIGNED_MALLOC   1

// Uncomment this define to enable debug output.
//#define LEPFLIR_ENABLE_DEBUG_OUTPUT     1
```

## Hookup Instructions

Make sure to hookup SPI lines MISO, MOSI, and CLK correctly (Due, Zero, etc. often use pins 50=MISO, 51=MOSI, 52=CLK, 53=CS, but one can always just use the ICSP header pins ICSP-1=MISO, ICSP-4=MOSI, ICSP-3=CLK). The CS (chip select) pin may be any digital output pin, with usage being active-low. The recommended VCC supply and logic level is 3.3v, but 5v also seems to work. The two issolated side power pins on breakout can be left disconnected. The minimum SPI transfer rate is 2MHz, which means one needs at least a 4MHz board, but realistically a 16MHz board is likely required given the processing work involved. The actual SPI transfer rate selected will be the first one below 20MHz given the SPI clock divider (processor speed /2, /4, /8, ..., /128).
