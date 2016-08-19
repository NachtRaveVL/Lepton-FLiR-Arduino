# Lepton-FLiR-Arduino
Arduino Library for the Lepton FLiR Thermal Camera Module.

**Lepton-FLiR-Arduino v0.9**

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

// Uncomment this define if wanting to exclude extended i2c functions from compilation.
//#define LEPFLIR_EXCLUDE_EXT_I2C_FUNCS   1

// Uncomment this define to enable debug output.
//#define LEPFLIR_ENABLE_DEBUG_OUTPUT     1
```

## Hookup Instructions

Make sure to hookup the module's SPI lines MISO, MOSI, CLK (aka SCK), and CS (aka SS) correctly (Due, Zero, ATmega, etc. often use pins 50=MISO, 51=MOSI, 52=SCK, 53=SS, but one can just simply use the ICSP header pins ICSP-1=MISO, ICSP-4=MOSI, ICSP-3=SCK, which are consistent across all boards). The module's MOSI line can simply be grounded since the module only uses SPI for slave-out data transfers (slave-in data transfers being ignored). The SS pin may be any digital output pin, with usage being active-low. The recommended VCC power supply and logic level is 3.3v, but 5v also seems to work. The two issolated power pins on the side of the module's breakout can safely be left disconnected. The minimum SPI transfer rate is ~2.2MHz, which means one needs at least an 8MHz processor, but more realistically a 16MHz processor is likely required given the processing work involved to resize/BLIT the final image. The actual SPI transfer rate selected will be the first rate equal to or below 20MHz given the SPI clock divider (processor speed /2, /4, /8, /16, ..., /128).

## Memory Footprint Note

Image storage mode affects the total memory footprint. Memory constrained boards should take notice to the storage requirements. Note that the Lepton FLiR delivers 14bpp thermal image data with AGC mode disabled and 8bpp thermal image data with AGC mode enabled, therefore if using AGC mode always enabled it is more memory efficient to use an 8bpp mode to begin with. Note that with telemetry enabled, memory cost incurs an additional 164 bytes for telemetry data storage. Lastly note that using the 80x60 16bpp mode is the most speed efficient since data transfers write directly to the image memory space without needing to perform software resizes/BLITs.

```Arduino
typedef enum {
    // Full 16bpp image mode, 9600 bytes for image data, 164 bytes for read frame (9604 bytes total, 9806 bytes if aligned)
    LeptonFLiR_ImageStorageMode_80x60_16bpp,
    // Full 8bpp image mode, 4800 bytes for image data, 164 bytes for read frame (4964 bytes total, 5006 bytes if aligned)
    LeptonFLiR_ImageStorageMode_80x60_8bpp,

    // Halved 16bpp image mode, 2400 bytes for image data, 328 bytes for read frame (2728 bytes total, 2782 bytes if aligned)
    LeptonFLiR_ImageStorageMode_40x30_16bpp,
    // Halved 8bpp image mode, 1200 bytes for image data, 328 bytes for read frame (1528 bytes total, 1814 bytes if aligned)
    LeptonFLiR_ImageStorageMode_40x30_8bpp,

    // Quartered 16bpp image mode, 600 bytes for image data, 656 bytes for read frame (1256 bytes total, 1446 bytes if aligned)
    LeptonFLiR_ImageStorageMode_20x15_16bpp,
    // Quartered 8bpp image mode, 300 bytes for image data, 656 bytes for read frame (956 bytes total, 1202 bytes if aligned)
    LeptonFLiR_ImageStorageMode_20x15_8bpp,

    LeptonFLiR_ImageStorageMode_Count
} LeptonFLiR_ImageStorageMode;
```
