# Lepton-FLiR-Arduino
Arduino Library for the Lepton FLiR Thermal Camera Module.

**Lepton-FLiR-Arduino v2.1.1**

Library to control a Lepton FLiR (forward looking infrared) thermal camera module from an Arduino-like board (Portenta/Teensy 3+/ESP32+ minimum).  
Licensed under the non-restrictive MIT license.

Created by NachtRaveVL, August 1st, 2016.

This library allows communication with boards running a Lepton FLiR thermal camera module. It provides a wide range of functionality from adjustable temperature display modes to exposing the full functionality of the thermal camera itself.

Made primarily for Arduino microcontrollers, but should work with PlatformIO, Espressif, Teensy, STM32, Pico, and others - although one might experience turbulence until the bug reports get ironed out.

Dependencies include: SoftI2CMaster (optional).

Parts of this library are derived from the Lepton FLiR software development SDK, Copyright 2011,2012,2013,2014 FLIR Systems - Commercial Vision Systems. These guys are really great (and sent us free cameras to test with) and so we ask that you check them out at: <https://lepton.flir.com/>.

Note that this library *requires* a fast microcontroller - on the order of hundreds of MHz - in order to process the SPI-based image data transfer under the set frame period of 26fps, or 38.46ms (save nothing about actually doing the advanced image processing work on said image after capture). See Section 4.2.2.3.2 of the datasheet concerning VoSPI synchronization for more information on how to optimize frame reads.

The datasheet for the IC is available at <https://lepton.flir.com/wp-content/uploads/2019/02/flir-lepton-engineering-datasheet-203.pdf>.  
Additional interface documentation is available at <https://www.flir.com/globalassets/imported-assets/document/flir-lepton-software-interface-description-document.pdf>.

*If you value the work that we do, our small team always appreciates a subscription to our [Patreon](www.patreon.com/nachtrave).*

## Supported Microcontrollers

Unfortunately during our testing back in 2016, largely due to SPI data transfer limitations using the Arduino SPI library (which apparently could be better), we were unable to successfully utilize any Arduino-specific microcontrollers, including the Due. However, as of more recently there seems to be a renewed interest in this library for the Teensy 3/4+, and now particularly the impressive Pico. As well, the ESP32 and ESP32-S are also slated for experimentation. V2 now includes an optional asynchronous SPI capture path for platforms whose SPI implementation exposes `SPI_HAS_TRANSFER_ASYNC`. On Teensy this uses the SPI library's DMA-backed asynchronous transfer support while keeping the existing blocking SPI reader as the default and fallback path.

As of this writing, we don't have an exact listing of which specific microcontrollers will work with this library, but we are currently rewriting core parts of the library as well as will be testing this library with various microcontrollers to see what kind of support we can muster. We will update this section in the future with boards we've tested and their support status of this library. A particular focus is being applied to Teensy 4+.

| Microcontroller | Clock Speed | SPI Clock Divisor | SPI Data Speed | Library Support? |
| :--- | :--- | :--- | :--- | :--- |
| Arduino Uno  | 8MHz | /2 | 4MHz | Not supported. |
| Arduino Mega | 16MHz | /2 | 8MHz | Not supported. |
| Arduino Nano | 16MHz | /2 | 8MHz | Not supported. |
| Arduino Zero | 48MHz | /4 | 12MHz | _tbt_ (Likely not supported) |
| Teensy LC | 48MHz | /4 | 12MHz | _tbt_ (Likely not supported) |
| Arduino Nano 33 | 64MHz | /4 | 16MHz | _tbt_ |
| Teensy 3.2 | 72MHz | /4 | 18MHz | _tbt_ (Likely supported) |
| Arduino Due | 84MHz | /5 <sup>1</sup> | 16.8MHz | _tbt_ |
| Teensy 3.5 | 120MHz | /8 | 15MHz | _tbt_ |
| ESP32 | 160MHz | /8 | 20MHz | _tbt_ (Likely supported) |
| Teensy 3.6 | 180MHz | /16 | 11.25 MHz | _tbt_ (Likely not supported) |
| ESP32/ESP32-S | 240MHz | /8 | 15MHz | _tbt_ |
| Teensy 4.0/4.1 | 600MHz | /32 | 18.75 | _tbt_ (Likely supported) |

Note<sup>1</sup>: Arduino Due allows for non-power-of-2 clock divisors.

## Library Setup

### Installation

The easiest way to install this library is to utilize the Arduino IDE library manager, or through a package manager such as PlatformIO. Otherwise, simply download this library and extract its files into a `Lepton-FLiR-Arduino` folder in your Arduino custom libraries folder, typically found in your `[My ]Documents\Arduino\libraries` folder (Windows), or `~/Documents/Arduino/libraries/` folder (Linux/OSX).

### Header Defines
 
There are several defines inside of the library's main header file that allow for more fine-tuned control of the library. You may edit and uncomment these lines directly, or supply them via custom build flags. While editing the main header file isn't ideal, it is often easiest. Note that editing the library's main header file directly will affect all projects compiled on your system using those modified library files.

Alternatively, you may also refer to <https://forum.arduino.cc/index.php?topic=602603.0> on how to define custom build flags manually via modifying the platform[.local].txt file. Note that editing such directly will affect all other projects compiled on your system using those modified platform framework files, but at least you keep those changes to the same place.

From LeptonFLiR.h:
```Arduino
// Uncomment or -D this define to enable usage of the software i2c library (min 4MHz+ processor).
//#define LEPFLIR_ENABLE_SOFTWARE_I2C             // https://github.com/felias-fogg/SoftI2CMaster

// Uncomment or -D this define to enable debug output (treats Serial output as attached to serial monitor).
//#define LEPFLIR_ENABLE_DEBUG_OUTPUT
```

### Library Initialization

There are several initialization mode settings exposed through this library that are used for more fine-tuned control.

#### Class Instantiation

The library's class object must first be instantiated, commonly at the top of the sketch where pin setups are defined (or exposed through some other mechanism), which makes a call to the library's class constructor. The constructor allows one to set the module's SPI CS pin, ISR VSync pin, i2c Wire class instance, and lastly i2c clock speed (all i2c parameters being ommitted when in software i2c mode). The default constructor values of the library, if left unspecified, is SPI CS pin `SS`, ISR VSync pin `DISABLED`, and i2c Wire class instance `Wire` @`400k`Hz.

From LeptonFLiR.h, in class LeptonFLiR, when in hardware i2c mode:
```Arduino
    // Library constructor. Typically called during class instantiation, before setup().
    // ISR VSync pin only available for Lepton FLiR breakout board v2+ (GPIO3=VSYNC).
    // Boards with more than one i2c line (e.g. Due/Teensy/etc.) can supply a different
    // Wire instance, such as Wire1 (using SDA1/SCL1), Wire2 (using SDA2/SCL2), etc.
    // Supported i2c clock speeds are 100kHz, 400kHz, and 1000kHz.
    // Supported SPI clock speeds are ~2.2MHz(@80x60)/~8.8MHz(@160x120) to 20MHz.
    LeptonFLiR(byte spiCSPin = SS, byte isrVSyncPin = DISABLED, TwoWire& i2cWire = Wire, uint32_t i2cSpeed = 400000);

    // Convenience constructor for custom Wire instance. See main constructor.
    LeptonFLiR(TwoWire& i2cWire, uint32_t i2cSpeed = 400000, byte spiCSPin = SS, byte isrVSyncPin = DISABLED);
```

From LeptonFLiR.h, in class LeptonFLiR, when in software i2c mode (see examples for sample usage):
```Arduino
    // Library constructor. Typically called during class instantiation, before setup().
    // ISR VSync pin only available for Lepton FLiR breakout board v2+ (GPIO3=VSYNC).
    // Minimum supported i2c clock speed is 100kHz, which sets minimum processor speed at
    // 4MHz+ running in i2c standard mode. For up to 400kHz i2c clock speeds, minimum
    // processor speed is 16MHz+ running in i2c fast mode.
    // Supported SPI clock speeds are ~2.2MHz(@80x60)/~8.8MHz(@160x120) to 20MHz.
    LeptonFLiR(byte spiCSPin = SS, byte isrVSyncPin = DISABLED);
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

* The recommended Vcc power supply and logic level is 3.3v.
* The two issolated power pins on the side of the FLiR v1.4 and v2 breakouts can safely be left disconnected.

### SPI Bus

SPI devices can be chained together on the same shared data lines, which are typically labeled `COPI` (or `MOSI`), `CIPO` (or `MISO`), and `SCK`, often with an additional `CS` (or `SS`). Each SPI device requires its own individual cable-select `CS` wire as only one SPI device may be active at any given time - accomplished by pulling its `CS` line of that device low (aka active-low). SPI runs at MHz speeds and is useful for large data block transfers.

* The `CS` pin may be connected to any digital output pin, but it's common to use the `CS` (or `SS`) pin for the first device. Additional devices are not restricted to what pin they can or should use, but given it's not a data pin not using a choice interrupt-capable pin allows those to be used for interrupt driven mechanisms.
* The module's `MOSI` line is optional and can simply be grounded since the module only uses SPI for slave-out data transfers (slave-in data being ignored).
* The minimum SPI transfer rate depends on the image resolution used by the camera, with 80x60 displays requiring ~2.2MHz minimum, and 160x120 displays requiring ~8.8MHz minimum, while the maximum SPI transfer rate is 20MHz.
  * The actual SPI transfer rate selected will be the first rate equal to or below 20MHz given the SPI clock divider (i.e. processor speed /2, /4, /8, ..., /128).
  * Anything below 12MHz is considered sub-optimal, and may have difficulty maintaining VoSPI syncronization.

### DMA SPI Capture

Boards whose SPI library exposes `SPI_HAS_TRANSFER_ASYNC` can use the alternate DMA-backed packet reader. Call `setSPIDMAEnabled()` after `init()`. If asynchronous SPI is unavailable, the method returns `false` and the existing blocking SPI path remains unchanged. If a DMA transfer cannot be started at runtime, capture automatically falls back to blocking SPI.

See [`examples/DMACaptureExample/DMACaptureExample.ino`](examples/DMACaptureExample/DMACaptureExample.ino) for a minimal setup.

### VSync Capture

Breakout boards exposing Lepton GPIO3 can optionally use the camera's VSync frame-timing pulse so `tryReadNextFrame()` only starts a read when a new frame is ready. Supply an interrupt-capable host pin as the constructor's VSync pin and configure Lepton GPIO3 for VSync through the existing OEM command interface. Only one `LeptonFLiR` instance may own the VSync ISR at a time.

```Arduino
const byte flirCSPin = 10;
const byte flirVSyncPin = 2;
LeptonFLiR flirController(flirCSPin, flirVSyncPin);

void setup() {
    SPI.begin();
    Wire.begin();
    Wire.setClock(flirController.getI2CSpeed());

    flirController.init(LeptonFLiR_CameraType_Lepton3_5);
    flirController.oem_setGPIOMode(LEP_OEM_GPIO_MODE_VSYNC);
}
```

Some Lepton firmware revisions document the GPIO mode setter but report it as unsupported. Check `getLastI2CError()` and `getLastLepResult()` immediately after the OEM call; on such firmware GPIO3 must already be configured for VSync. When a VSync pin is supplied but no VSync pulse arrives, `tryReadNextFrame()` returns `false` without beginning an SPI frame read.

### I2C Bus

I2C (aka I²C, IIC, TwoWire, TWI) devices can be chained together on the same shared bus lines (no flipping of wires), which are typically labeled `SCL` and `SDA`. Only different kinds of I2C devices can be used on the same bus line together using factory default settings, otherwise manual addressing must be done. I2C runs at mid to high kHz speeds and is useful for advanced device control.

* When more than one I2C device of the same kind is to be used on the same bus line, each device must be set to use a different address. This is accomplished via the A0-A2 (sometimes A0-A5) pins/pads on the physical device that must be set either open or closed (typically via a de-solderable resistor, or by shorting a pin/pad). Check your specific breakout's datasheet for details.
* Note that not all the I2C libraries used support multi-addressable I2C devices at this time. Currently, this restriction applies to RTC devices (read as: may only use one).

## Memory Callouts

### Memory Storage Cost

Which Lepton camera version is being used and which color mode(s) is(are) active will determine the memory requirements to store a single video image frame, and must be set at initialization time. _Future versions of this library hope to automatically detect such._ Leptons before v3 commonly use a 80x60 pixel frame, while Leptons of v3 and later use a 160x120 pixel frame. Keep in mind that higher image size costs not just extra memory, but also more data that needs transfered over SPI. SPI data transfer must succeed in under a set frame time, and if such does not occur (referred to as a de-sync) then the rest of the data frame is lost - this has long been the major bottleneck of using this module with less powerful microcontrollers in the past.

### Image Color Mode

The various ways in which image data is stored, and thus accessed, is based on the following:
* When neither AGC (automatic gain correction), TLinear (aka radiometric output), nor pseudo-color LUT (aka palettized) modes are enabled, the image data will be in 16bpp grayscale mode with the 2 most-signifcant bits zero'ed out (effectively 14bpp) - this is considered the standard run mode.
* When TLinear (aka radiometric output) mode is enabled, the image data will be in 16bpp grayscale mode (full 16bpp).
* When AGC (automatic gain correction) mode is enabled, the default output is 8-bit grayscale carried in 16-bit VoSPI words. On Lepton 1.5-2.5, HEQ can instead output 14-bit grayscale when selected with `agc_setHEQScaleFactor(LEP_AGC_SCALE_TO_14_BITS)`. The library preserves this as `GS16` output; use `std.value` for its pixels. For `GS8` output, use `agc.value`.
* When pseudo-color LUT (aka palettized) mode is enabled, the image data will be 24bpp RGB888 (created from either the selected preset LUT or user-supplied LUT).

Due to the packet-nature of the VoSPI image data transfer and the desire to limit memory storage cost, transferring the image data out of the storage buffers requires special handling. Image data should be accessed through the supplied library functions so packet layout, telemetry, and Lepton v3+ segmented 160x120 frames are handled consistently. The frame reader currently decodes RAW14 and RGB888 VoSPI output; other VID/OEM output formats remain available through their corresponding camera command APIs but are not decoded by `tryReadNextFrame()`.

### Working with Captured Frames

After `tryReadNextFrame()` succeeds, the frame accessors operate on the frame that was just captured:

* Use `getImagePixelData(row, col)` to read an individual pixel without handling VoSPI packet headers or Lepton v3+ segmentation yourself. The active frame settings determine which union member is valid: `std.value`, `agc.value`, `tlinear.value`, or `pclut.red/green/blue`.
* Use `getImageOutputData()` to obtain the complete processed image as a normal row-oriented buffer. `getImageOutputBpp()`, `getImageOutputPitch()`, and `getImageOutputTotalSize()` describe that buffer. An overload is also available for copying into a caller-owned buffer. `getImageBpp()` and `getImageOutputBpp()` historically return **bytes per pixel** (1, 2, or 3), despite the `Bpp` name.
* Use the telemetry convenience getters for common fields or `getTelemetryOutputData()` for the complete processed telemetry structure.

Methods such as `getAGCEnabled()`, `getTLinearEnabled()`, and `getTelemetryMode()` describe the **last captured frame**. Module-prefixed methods such as `agc_getAGCEnabled()`, `rad_getTLinearEnabled()`, and `sys_getTelemetryEnabled()` query the camera's **current configuration**. This distinction matters when settings are changed between frames.

For radiometric Lepton 2.5/3.5 modules, TLinear pixels may use either 0.1 K or 0.01 K units. `rad_getTLinearResolution()` reports the configured resolution, while `kelvin100ToTemperature()` expects Kelvin x100. For example:

```Arduino
LeptonFLiR_PixelData pixel = flirController.getImagePixelData(row, col);
uint32_t kelvin100 = pixel.tlinear.value; // Keep rescaled high temperatures above 655.35 K intact

if (flirController.rad_getTLinearResolution() == LEP_RAD_RESOLUTION_0_1)
    kelvin100 *= 10;

float temperature = flirController.kelvin100ToTemperature(kelvin100);
```

### Checking Camera Command Results

Module command methods preserve the most recent I2C transport error and Lepton command result. Check these immediately after a command when failure matters, before issuing another camera command that would replace the stored result:

```Arduino
flirController.rad_setTLinearEnabled(ENABLED);

if (flirController.getLastI2CError() || flirController.getLastLepResult() != LEP_OK) {
    // Camera command failed.
}
```

## Example Usage

Below are several examples of library usage.

### Simple Example

See [`examples/SimpleExample/SimpleExample.ino`](examples/SimpleExample/SimpleExample.ino) for the complete sketch.

### Advanced Example

In this example, we will utilize various features of the library after capturing a frame: inspect its dimensions and mode, access an individual pixel, obtain the complete processed image buffer, and read processed telemetry.

We will be using Wire1, which is only available on boards with SDA1/SCL1 (e.g. Due/Teensy/etc.) - change to Wire if Wire1 is unavailable.

See [`examples/AdvancedExample/AdvancedExample.ino`](examples/AdvancedExample/AdvancedExample.ino) for the complete sketch.

### Image Capture Example

In this example, we will copy out thermal image frames to individual BMP files located on a MicroSD card using the SD library.

Note that you will need a MicroSD card reader module for this example to work. Both the FLiR module and MicroSD card reader module will be on the same SPI lines, just using different chip enable pins/wires.

See [`examples/ImageCaptureExample/ImageCaptureExample.ino`](examples/ImageCaptureExample/ImageCaptureExample.ino) for the complete sketch.

### Software i2c Example

In this example, we utilize a popular software i2c library for chips that do not have a hardware i2c bus, available at <https://github.com/felias-fogg/SoftI2CMaster>.

If one uncomments the line below inside the main header file (or defines it via custom build flag), software i2c mode for the library will be enabled. Additionally, you will need to correctly define SCL_PIN, SCL_PORT, SDA_PIN, and SDA_PORT according to your setup. I2C_FASTMODE=1 should be set for 16MHz+ processors. Lastly note that, while in software i2c mode, the i2c clock speed returned by the library (via `getI2CSpeed()`) is only an upper bound and may not represent the actual i2c clock speed set nor achieved.

In LeptonFLiR.h:
```Arduino
// Uncomment or -D this define to enable usage of the software i2c library (min 4MHz+ processor).
#define LEPFLIR_ENABLE_SOFTWARE_I2C             // https://github.com/felias-fogg/SoftI2CMaster
```  
Alternatively, in platform[.local].txt:
```Arduino
build.extra_flags=-DLEPFLIR_ENABLE_SOFTWARE_I2C
```

In main sketch:
See [`examples/SoftwareI2CExample/SoftwareI2CExample.ino`](examples/SoftwareI2CExample/SoftwareI2CExample.ino) for the complete sketch.

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
See [`examples/ModuleInfo/ModuleInfo.ino`](examples/ModuleInfo/ModuleInfo.ino) for the complete sketch.

The serial monitor will show the camera configuration, AGC/SYS/VID state, OEM part and software information, and radiometry state on supported radiometric models.


## Specification Fixes and Host Tests

The specification regression tests cover Engineering Datasheet Rev 203 telemetry fields,
Software IDD Rev 303 HEQ scale selection, rejected CCI commands, string termination,
and VoSPI recovery. The software-I2C tests exercise address/data NACKs, byte order,
and final-byte NACK/STOP handling through the actual library transport code.
The follow-up regression tests exercise image capture filenames, settings refresh after
reboot/default restore, full-range temperature thresholds, and command timer rollover.

When VSYNC capture loses synchronization, the reader holds CS high and leaves SCK idle
for 186 ms, then requires a fresh VSYNC pulse. Successful captures do not incur this delay.
Telemetry strings now have room for all hexadecimal digits plus NUL (33 bytes for the
serial number, 17 for software revision). The existing FFC enum values remain unchanged;
`LeptonFLiR_TelemetryFFCState_Imminent` is appended. Rebuild code using the telemetry structure.
Always check `getLastI2CError()` and `getLastLepResult()` after camera commands; a failed
GET does not make its output valid.

`kelvin100ToTemperature()` accepts a 32-bit value so high-temperature telemetry and
rescaled 0.1 K TLinear pixels do not overflow a 16-bit intermediate. Reboot and user
defaults restore invalidate the cached settings for the next captured frame; wait for
the camera to be ready after reboot before resuming capture.

Run the seven host test suites and compile the examples with:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Host tests use simulated bus operations; they do not establish physical camera timing
or board-specific DMA compatibility.
