# Lepton-FLiR-Arduino
Arduino Library for the Lepton FLiR Thermal Camera Module.

**Lepton-FLiR-Arduino v0.5**

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

// Uncomment this define to disable 16-byte aligned memory allocations (may hinder performance).
//#define LEPFLIR_DISABLE_ALIGNED_MALLOC  1

// Uncomment this define to enable debug output.
//#define LEPFLIR_ENABLE_DEBUG_OUTPUT     1
```

## Hookup Instructions

Make sure to hookup the module's SPI lines MISO, MOSI, CLK (aka SCK), and CS (aka SS) correctly (Due, Zero, ATmega, etc. often use pins 50=MISO, 51=MOSI, 52=SCK, 53=SS, but one can just simply use the ICSP header pins ICSP-1=MISO, ICSP-4=MOSI, ICSP-3=SCK, which are consistent across all boards). The module's MOSI line can simply be grounded since the module only uses SPI for slave-out data transfers (slave-in data transfers being ignored). The SS pin may be any digital output pin, with usage being active-low. The recommended VCC power supply and logic level is 3.3v, but 5v also seems to work. The two issolated power pins on the side of the module's breakout can safely be left disconnected. The minimum SPI transfer rate is ~2.2MHz, which means one needs at least an 8MHz processor, but more realistically a 16MHz processor is likely required given the processing work involved to resize/BLIT the final image. The actual SPI transfer rate selected will be the first rate equal to or below 20MHz given the SPI clock divider (processor speed /2, /4, /8, /16, ..., /128).
