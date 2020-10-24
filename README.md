# Lepton-FLiR-Arduino
Arduino Library for the Lepton FLiR Thermal Camera Module.

**NOTE: SPI data transfer mechanism is still being ironed out. Check back later when v1.0 releases.**

**Lepton-FLiR-Arduino v0.9.92**

**UNDER RENEWED DEVELOPMENT AS OF AUGUST 2020**

Library to control a Lepton FLiR (forward looking infrared) thermal camera module from an Arduino-like board (Teensy 3+/ESP32+ minimum).  
Licensed under the non-restrictive MIT license.

Created by NachtRaveVL, August 1st, 2016.

This library allows communication with boards running a Lepton FLiR thermal camera module. It provides a wide range of functionality from adjustable temperature display mode, fast chip select enable/disable routines, to exposing the full functionality of the thermal camera itself.

Made primarily for Arduino microcontrollers, but should work with PlatformIO, ESP32/8266, Teensy, and others - although one might want to ensure `BUFFER_LENGTH` (or `I2C_BUFFER_LENGTH`) and `WIRE_INTERFACES_COUNT` is properly defined for any architecture used.

Dependencies include Scheduler if on a ARM/ARMD architecture (e.g. Due/Zero/etc.), but usage can be disabled via library setup header defines or custom build flags.

Parts of this library are derived from the Lepton FLiR software development SDK, Copyright 2011,2012,2013,2014 FLIR Systems - Commercial Vision Systems.

Note that this library *requires* a fast microcontroller - on the order of hundreds of MHz - in order to process the SPI-based image data transfer under the set frame period of 26fps, or 38.46ms. See Section 4.2.2.3.2 of the datasheet concerning VoSPI synchronization for more information.

The datasheet for the IC is available at <https://lepton.flir.com/wp-content/uploads/2019/02/flir-lepton-engineering-datasheet-203.pdf>.  
Additional interface documentation is available at <https://www.flir.com/globalassets/imported-assets/document/flir-lepton-software-interface-description-document.pdf>.

## Supported Microcontrollers

Unfortunately during our testing back in 2016, largely due to SPI data transfer limitations using the Arduino SPI library (which apparently is shite), we were unable to successfully utilize any Arduino-specific microcontrollers, including the Due. However, as of more recently there seems to be a renewed interest in this library for the Teensy 3, and now particularly the impressive 600MHz Teensy 4. As well, the ESP32 and ESP32-S are also slated for experimentation. Additionally, we plan on experimenting with a faster custom SPI transfer interface to try and overcome speed limitations on slower microcontrollers, and might be able to look into DMA options available on the Arduino Zero and Arduino Portenta.

As of this writing, we don't have an exact listing of which specific microcontrollers will work with this library, but we are currently rewriting core parts of the library as well as will be testing this library with various microcontrollers to see what kind of support we can muster. We will update this section in the future with boards we've tested and their support status of this library. A particular focus is being applied to Teensy 4+.

| Microcontroller | Clock Speed | SPI Clock Divisor | SPI Data Speed | Library Support? |
| :--- | :--- | :--- | :--- | :--- |
| Arduino Uno  | 8MHz | /2 | 4MHz | Not supported. |
| Arduino Mega | 16MHz | /2 | 8MHz | Not supported. |
| Arduino Nano | 16MHz | /2 | 8MHz | Not supported. |
| Arduino Zero | 48MHz | /4 | 12MHz | _tbt_ (Likely not supported) |
| Teensy LC | 48MHz | /4 | 12MHz | _tbt_ (Likely not supported) |
| Arduino Nano 33 | 64MHz | /4 | 16MHz |  _tbt_ |
| Teensy 3.2 | 72MHz | /4 | 18MHz | _tbt_ |
| Arduino Due | 84MHz | /5 <sup>1</sup> | 16.8MHz | _tbt_ |
| Teensy 3.5 | 120MHz | /8 | 15MHz | _tbt_ |
| ESP32 | 160MHz | /8 | 20MHz | _tbt_ |
| Teensy 3.6 | 180MHz | /16 | 11.25 MHz | _tbt_ |
| ESP32/ESP32-S | 240MHz | /8 | 15MHz | _tbt_ (Possibly supported?) |
| Teensy 4.0/4.1 | 600MHz | /32 | 18.75 | _tbt_ (Likely supported?) |

Note<sup>1</sup>: Arduino Due allows for non-power-of-2 clock divisors.

## Library Setup

### Installation

The easiest way to install this library is to utilize the Arduino IDE library manager, or through a package manager such as PlatformIO. Otherwise, simply download this library and extract its files into a `Lepton-FLiR-Arduino` folder in your Arduino custom libraries folder, typically found in your `[My ]Documents\Arduino\libraries` folder (Windows), or `~/Documents/Arduino/libraries/` folder (Linux/OSX).

### Header Defines
 
There are several defines inside of the library's main header file that allow for more fine-tuned control of the library. You may edit and uncomment these lines directly, or supply them via custom build flags. While editing the main header file isn't ideal, it is often the easiest given the Arduino IDE's limited custom build flag support. Note that editing the library's main header file directly will affect all projects compiled on your system using those modified library files.

Alternatively, you may also refer to <https://forum.arduino.cc/index.php?topic=602603.0> on how to define custom build flags manually via modifying the platform[.local].txt file. Note that editing such directly will affect all other projects compiled on your system using those modified platform framework files.

From LeptonFLiR.h:
```Arduino
// Uncomment or -D this define to enable use of the software i2c library (min 4MHz+ processor).
//#define LEPFLIR_ENABLE_SOFTWARE_I2C             // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment or -D this define to disable usage of the Scheduler library on SAM/SAMD architecures.
//#define LEPFLIR_DISABLE_SCHEDULER               // https://github.com/arduino-libraries/Scheduler

// Uncomment or -D this define to enable debug output.
//#define LEPFLIR_ENABLE_DEBUG_OUTPUT
```

### Library Initialization

There are several initialization mode settings exposed through this library that are used for more fine-tuned control.

#### Class Instantiation

The library's class object must first be instantiated, commonly at the top of the sketch where pin setups are defined (or exposed through some other mechanism), which makes a call to the library's class constructor. The constructor allows one to set the module's SPI CS pin, ISR VSync pin, i2c Wire class instance, if on Espressif then i2c SDA pin and i2c SCL pin, and lastly i2c clock speed (all i2c parameters being ommitted when in software i2c mode). The default constructor values of the library, if left unspecified, is SPI CS pin `D10`, ISR VSync pin `DISABLED`, i2c Wire class instance `Wire` @`400k`Hz, and if on Espressif then i2c SDA pin `D21` and i2c SCL pin `D22` (ESP32[-S] defaults).

From LeptonFLiR.h, in class LeptonFLiR, when in hardware i2c mode:
```Arduino
    // Library constructor. Typically called during class instantiation, before setup().
    // ISR VSync pin only available for Lepton FLiR breakout board v2+ (GPIO3=VSYNC).
    // Boards with more than one i2c line (e.g. Due/Teensy/etc.) can supply a different
    // Wire instance, such as Wire1 (using SDA1/SCL1), Wire2 (using SDA2/SCL2), etc.
    // Supported i2c clock speeds are 100kHz, 400kHz, and 1000kHz.
    // Supported SPI clock speeds are ~2.2MHz(@80x60)/~8.8MHz(@160x120) to 20MHz.
    LeptonFLiR(byte spiCSPin = 10, byte isrVSyncPin = DISABLED, TwoWire& i2cWire = Wire, uint32_t i2cSpeed = 400000);

    // Convenience constructor for custom Wire instance. See main constructor.
    LeptonFLiR(TwoWire& i2cWire, uint32_t i2cSpeed = 400000, byte spiCSPin = 10, byte isrVSyncPin = DISABLED);
```

From LeptonFLiR.h, in class LeptonFLiR, when in software i2c mode (see examples for sample usage):
```Arduino
    // Library constructor. Typically called during class instantiation, before setup().
    // ISR VSync pin only available for Lepton FLiR breakout board v2+ (GPIO3=VSYNC).
    // Minimum supported i2c clock speed is 100kHz, which sets minimum processor speed at
    // 4MHz+ running in i2c standard mode. For up to 400kHz i2c clock speeds, minimum
    // processor speed is 16MHz+ running in i2c fast mode.
    // Supported SPI clock speeds are ~2.2MHz(@80x60)/~8.8MHz(@160x120) to 20MHz.
    LeptonFLiR(byte spiCSPin = 10, byte isrVSyncPin = DISABLED);
```

#### Device Initialization

Additionally, a call is expected to be provided to the library class object's `init(…)` method, commonly called inside of the sketch's `setup()` function. This allows one to set the module's hardware camera type and temperature display mode. The module's hardware camera type must be explicitly provided. The remaining default init values of the library, if left unspecified, is `LeptonFLiR_TemperatureMode_Celsius`.

From LeptonFLiR.h, in class LeptonFLiR:
```Arduino
    // Initializes module. Typically called in setup().
    // See individual enums for more info.
    void init(LeptonFLiR_CameraType cameraType, LeptonFLiR_TemperatureMode tempMode = LeptonFLiR_TemperatureMode_Celsius);
```

From LeptonFLiRDefines.h:
```Arduino
enum LeptonFLiR_CameraType {
    LeptonFLiR_CameraType_Lepton1,              // Lepton v1 camera, running at 80x60
    LeptonFLiR_CameraType_Lepton1_5,            // Lepton v1.5 camera, running at 80x60
    LeptonFLiR_CameraType_Lepton1_6,            // Lepton v1.6 camera, running at 80x60
    LeptonFLiR_CameraType_Lepton2,              // Lepton v2 camera, running at 80x60
    LeptonFLiR_CameraType_Lepton2_5,            // Lepton v2.5 camera, running at 80x60
    LeptonFLiR_CameraType_Lepton3,              // Lepton v3 camera, running at 160x120
    LeptonFLiR_CameraType_Lepton3_5,            // Lepton v3.5 camera, running at 160x120
};

enum LeptonFLiR_TemperatureMode {
    LeptonFLiR_TemperatureMode_Celsius,         // Celsius temperature mode
    LeptonFLiR_TemperatureMode_Fahrenheit,      // Fahrenheit temperature mode
    LeptonFLiR_TemperatureMode_Kelvin,          // Kelvin temperature mode
};
```

## Hookup Callouts

* The recommended VCC power supply and logic level is 3.3v.
* The two issolated power pins on the side of the FLiR v1.4 and v2 breakouts can safely be left disconnected.

### SPI Data Line

* Make sure to hookup the module's SPI lines MISO, MOSI, CLK (aka SCK), and CS (aka SS) correctly. Teensy 3.X uses pins 12=MISO, 11=MOSI, 13=SCK, and 10=SS, while ESP32[-S] uses pins 19=MISO, 23=MOSI, 16=SCK, and 5=SS.
* The module's MOSI line is optional and can simply be grounded since the module only uses SPI for slave-out data transfers (slave-in data transfers being ignored).
* The SS pin may be any digital output pin, with usage being active-low.
* The minimum SPI transfer rate depends on the image resolution used by the camera, with 80x60 displays requiring ~2.2MHz minimum, and 120x60 displays requiring ~8.8MHz minimum, while the maximum SPI transfer rate is 20MHz.
  * The actual SPI transfer rate selected will be the first rate equal to or below 20MHz given the SPI clock divider (i.e. processor speed /2, /4, /8, ..., /128).
  * Anything below 12MHz is considered sub-optimal, and may have difficulty maintaining VoSPI syncronization.

## Memory Callouts

### Memory Storage Cost

Which Lepton camera version is being used and which color mode(s) is(are) active will determine the memory requirements to store a single video image frame, and must be set at initialization time. _Future versions of this library hope to automatically detect such._ Leptons before v3 commonly use a 80x60 pixel frame, while Leptons of v3 and later use a 160x120 pixel frame. Keep in mind that higher image size costs not just extra memory, but also more data that needs transfered over SPI. SPI data transfer must succeed in under a set frame time, and if such does not occur (referred to as a de-sync) then the rest of the data frame is lost - this has long been the major bottleneck of using this module with less powerful microcontrollers in the past.

### Image Color Mode

The various ways in which image data is stored, and thus accessed, is based on the following:
* When neither AGC (automatic gain correction), TLinear (aka radiometric output), nor pseudo-color LUT (aka palettized) modes are enabled, the image data will be in 16bpp grayscale mode with the 2 most-signifcant bits zero'ed out (effectively 14bpp) - this is considered the standard run mode.
* When TLinear (aka radiometric output) mode is enabled, the image data will be in 16bpp grayscale mode (full 16bpp).
* When AGC (automatic gain correction) mode is enabled, the image data will be in 16bpp grayscale mode with the 8 most-significant bits being zero'ed out (effectively 8bbp).
* When pseudo-color LUT (aka palettized) mode is enabled, the image data will be 24bpp RGB888 (created from either the selected preset LUT or user-supplied LUT).

Due to the packet-nature of the VoSPI image data transfer and the desire to limit memory storage cost, transfering the image data out of the storage buffers requires special handling. Image row data must be accessed via the supplied library functions, largely since SPI packet data may arrive out-of-order. In 160x120 frame size mode (Lepton v3+), rows are split into two sections and which section being accessed must be specified. _Future versions of this library will provide a more robust way of supporting final image access, as well as support for Lepton v3+ running 160x120 frame size mode._

## Example Usage

Below are several examples of library usage.

### Simple Example

```Arduino
// TODO: Reinclude this example after modifications completed. -NR
```

### Advanced Example

In this example, we will utilize various features of the library.

We will be using Wire1, which is only available on boards with SDA1/SCL1 (e.g. Due/Teensy/etc.) - change to Wire if Wire1 is unavailable. We will also be using the digitalWriteFast library, available at <https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast>.

```Arduino
// TODO: Reinclude this example after modifications completed. -NR
```

### Image Capture Example

In this example, we will copy out thermal image frames to individual BMP files located on a MicroSD card using the SD library.

Note that you will need a MicroSD card reader module for this example to work. Both the FLiR module and MicroSD card reader module will be on the same SPI lines, just using different chip enable pins/wires.

```Arduino
// TODO: Reinclude this example after modifications completed. -NR
```

### Software i2c Example

In this example, we utilize a popular software i2c library for chips that do not have a hardware i2c bus, available at <http://playground.arduino.cc/Main/SoftwareI2CLibrary>.

If one uncomments the line below inside the main header file (or defines it via custom build flag), software i2c mode for the library will be enabled. Additionally, you will need to correctly define SCL_PIN, SCL_PORT, SDA_PIN, and SDA_PORT according to your setup. I2C_FASTMODE=1 should be set for 16MHz+ processors. Lastly note that, while in software i2c mode, the i2c clock speed returned by the library (via `getI2CSpeed()`) is only an upper bound and may not represent the actual i2c clock speed set nor achieved.

In LeptonFLiR.h:
```Arduino
// Uncomment or -D this define to enable use of the software i2c library (min 4MHz+ processor).
#define LEPFLIR_ENABLE_SOFTWARE_I2C             // http://playground.arduino.cc/Main/SoftwareI2CLibrary
```  
Alternatively, in platform[.local].txt:
```Arduino
build.extra_flags=-DLEPFLIR_ENABLE_SOFTWARE_I2C
```

In main sketch:
```Arduino
// TODO: Reinclude this example after modifications completed. -NR
```

## Module Info

In this example, we enable debug output support to print out module diagnostic information.

If one uncomments the line below inside the main header file (or defines it via custom build flag), debug output support will be enabled and the printModuleInfo() method will become available. Calling this method will display information about the module itself, including initalized states, register values, current settings, etc. Additionally, all library calls being made will display internal debug information about the structure of the call itself. An example of this output is shown below.

In LeptonFLiR.h:
```Arduino
// Uncomment or -D this define to enable debug output.
#define LEPFLIR_ENABLE_DEBUG_OUTPUT
```  
Alternatively, in platform[.local].txt:
```Arduino
build.extra_flags=-DLEPFLIR_ENABLE_DEBUG_OUTPUT
```

In main sketch:
```Arduino
// TODO: Reinclude this example after modifications completed. -NR
```

In serial monitor:
```
 // TODO: Reinclude this example output after modifications completed. -NR
```
