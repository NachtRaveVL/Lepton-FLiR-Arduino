# Lepton-FLiR-Arduino
Arduino Library for the Lepton FLiR Thermal Camera Module.

**NOTE: SPI data transfer mechanism is still being ironed out. Check back later when v1.0 releases.**

**Lepton-FLiR-Arduino v0.9.91**

**UNDER RENEWED DEVELOPMENT AS OF AUGUST 2020**

Library to control a Lepton FLiR (forward looking infrared) thermal camera module from an Arduino-like board (Teensy 3+/ESP32+ minimum).  
Licensed under the non-restrictive MIT license.

Created by NachtRaveVL, August 1st, 2016.

This library allows communication with boards running a Lepton FLiR thermal camera module. It provides a wide range of functionality from adjustable temperature display mode, fast chip select enable/disable routines, to exposing the full functionality of the thermal camera itself.

Parts of this library are derived from the Lepton FLiR software development SDK, Copyright 2011,2012,2013,2014 FLIR Systems - Commercial Vision Systems.

The datasheet for the IC is available from <https://lepton.flir.com/wp-content/uploads/2019/02/flir-lepton-engineering-datasheet-203.pdf>.  
Additional interface documentation is available from <https://www.flir.com/globalassets/imported-assets/document/flir-lepton-software-interface-description-document.pdf>.

## Supported Microcontrollers

Unfortunately during our testing, largely due to SPI data transfer limitations, we were unable to successfully utilize any Arduino-specific microcontrollers, including the Due (_sad face_). However, as of more recently there seems to be a renewed interest in this library for the Teensy 3, and now particularly the impressive 600MHz Teensy 4. As well, the ESP32 and ESP32-S are also slated for experimentation.

As of this writing, we don't have the exact list of which specific microcontrollers will work with this library, but we are currently testing this library under those systems and seeing what supported we can muster. We will update this section in the future with boards we've tested this library on.

## Library Setup

### Header Defines

There are several defines inside of the library's main header file that allow for more fine-tuned control of the library. You may edit and uncomment these lines directly, or supply them as a compilation flag via custom build system. While editing the main header file isn't the most ideal, it is often the easiest way when using the Arduino IDE, as it doesn't support custom build flags. Be aware that editing this header file directly will affect all projects on your system using this library.

In LeptonFLiR.h:
```Arduino
// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
//#define LEPFLIR_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define to disable 16 byte aligned memory allocations (may hinder performance).
//#define LEPFLIR_DISABLE_ALIGNED_MALLOC  1

// Uncomment this define if wanting to exclude extended i2c functions from compilation.
//#define LEPFLIR_EXCLUDE_EXT_I2C_FUNCS   1

// Uncomment this define to enable debug output.
//#define LEPFLIR_ENABLE_DEBUG_OUTPUT     1
```

### Library Initialization

There are several initialization mode flags exposed through this library that are used for more fine-tuned control. These flags are expected to be provided to the library's `init(...)` function, commonly called inside of the sketch's `setup()` function.

From LeptonFLiR.h:
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
} LeptonFLiR_ImageStorageMode;

typedef enum {
    LeptonFLiR_TemperatureMode_Celsius,
    LeptonFLiR_TemperatureMode_Fahrenheit,
    LeptonFLiR_TemperatureMode_Kelvin,
} LeptonFLiR_TemperatureMode;
```

## Hookup Callouts

### SPI Data Line

Make sure to hookup the module's SPI lines MISO, MOSI, CLK (aka SCK), and CS (aka SS) correctly. Teensy 3.X uses pins 12=MISO, 11=MOSI, 13=SCK, and 10=SS, while ESP32X uses pins 19=MISO, 23=MOSI, 16=SCK, and 5=SS. The module's MOSI line is optional and can simply be grounded since the module only uses SPI for slave-out data transfers (slave-in data transfers being ignored). The SS pin may be any digital output pin, with usage being active-low. The recommended VCC power supply and logic level is 3.3v. The two issolated power pins on the side of the FLiR v1.4 breakout can safely be left disconnected. The minimum SPI transfer rate is ~2.2MHz, while the maximum SPI transfer rate is 20MHz. The actual SPI transfer rate selected will be the first rate equal to or below 20MHz given the SPI clock divider (i.e. processor speed /2, /4, /8, ..., /128).

## Memory Callouts

### Memory Storage Cost

Which Lepton camera version is being used and which color mode(s) is(are) active will determine the memory requirements to store a single video image frame, and must be set at initialization time. _Future versions of this library hope to automatically detect such._ Leptons before v3 commonly use a 80x60 pixel frame, while Leptons of v3 and later use a 160x120 pixel frame. Keep in mind that higher image size costs not just extra memory, but also more data that needs transfered over SPI. SPI data transfer must succeed in under a set frame time, and if such does not occur (referred to as a de-sync) then the rest of the data frame is lost - this has long been the major bottleneck of using this module with less powerful microcontrollers in the past.

### Image Color Mode

The various ways in which image data is stored, and thus accessed, is based on the following:
* When neither AGC (automatic gain correction), TLinear (aka radiometric output), nor pseudo-color LUT (aka palettized) modes are enabled, the image data will be in 16bpp grayscale mode with the 2 most-signifcant bits zero'ed out (effectively 14bpp) - this is considered the standard run mode.
* When TLinear (aka radiometric output) mode is enabled, the image data will be in 16bpp grayscale mode (full 16bpp).
* When AGC (automatic gain correction) mode is enabled, the image data will be in 16bpp grayscale mode with the 8 most-significant bits being zero'ed out (effectively 8bbp).
* When pseudo-color LUT (aka palettized) mode is enabled, the image data will be 24bpp RGB888 (created from either the selected preset LUT or user-supplied LUT).

Due to the packet-nature of the VoSPI transfer, transfering the image data out of the storage buffers requires special handling. _Future versions of this library will provide a more robust way of supporting final image access._

### Extended Functions

This library has an extended list of functionality for those who care to dive into such, but isn't always particularly the most useful for various general use cases. If one uncomments the line below inside the main header file (or defines it via custom build flag), this extended functionality can be manually compiled-out.

In LeptonFLiR.h:
```Arduino
// Uncomment this define if wanting to exclude extended i2c functions from compilation.
#define LEPFLIR_EXCLUDE_EXT_I2C_FUNCS   1
```

## Example Usage

Below are several examples of library usage.

### Simple Example

```Arduino
#include "LeptonFLiR.h"

LeptonFLiR flirController();            // Library using default Wire and default chip select pin D53

void setup() {
    Serial.begin(115200);

    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz
    SPI.begin();                        // SPI must be started first as well

    // Using 40x30 8bpp memory allocation mode and default celsius temperature mode
    flirController.init(LeptonFLiR_ImageStorageMode_40x30_8bpp);
}

void loop() {
    flirController.readNextFrame();     // Read next frame and store result into internal imageData
}

```

### Advanced Example

In this example, we will utilize various features of the library.

We will be using Wire1, which is only available on boards with SDA1/SCL1 (Due, Zero, etc.) - change to Wire if Wire1 is unavailable. We will also be using the digitalWriteFast library, available at: https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

```Arduino
#include "LeptonFLiR.h"
#include "digitalWriteFast.h"

const byte flirCSPin = 22;
LeptonFLiR flirController(Wire1, flirCSPin); // Library using Wire1 and chip select pin D22

// Fast CS enable/disable routines, using the digitalWriteFast library
static void fastEnableCS(byte pin) { digitalWriteFast(pin, LOW); }
static void fastDisableCS(byte pin) { digitalWriteFast(pin, HIGH); }

void setup() {
    Serial.begin(115200);

    Wire1.begin();                      // Wire1 must be started first
    Wire1.setClock(400000);             // Supported baud rates are 100kHz, 400kHz, and 1000kHz
    SPI.begin();                        // SPI must be started first as well

    // Using default memory allocation mode 80x60 16bpp and default celsius temperature mode
    flirController.init();

    // Setting use of fast enable/disable methods for chip select
    flirController.setFastCSFuncs(fastEnableCS, fastDisableCS);

    flirController.sys_setTelemetryEnabled(ENABLED); // Ensure telemetry is enabled
}

void loop() {
    if (flirController.readNextFrame()) { // Read next frame and store result into internal imageData
        // Find the hottest spot on the frame
        int hotVal = 0; hotX, hotY;

        for (int y = 0; y < flirController.getImageHeight(); ++y) {
            for (int x = 0; x < flirController.getImageWidth(); ++x) {
                int val = flirController.getImageDataRowCol(y, x);

                if (val > hotVal) {
                    hotVal = val;
                    hotX = x; hotY = y;
                }
            }
        }

        Serial.print("Hottest point: [");
        Serial.print(hotX);
        Serial.print(",");
        Serial.print(hotY);
        Serial.println("]");
    }
}

```

### Image Capture Example

In this example, we will copy out thermal image frames to individual BMP files located on a MicroSD card using the SD library.

Note that you will need a MicroSD card reader module for this example to work. Both the FLiR module and MicroSD card reader module will be on the same SPI lines, just using different chip enable pins/wires.

```Arduino
#include "LeptonFLiR.h"
#include <SD.h>

const byte flirCSPin = 22;
LeptonFLiR flirController(Wire, flirCSPin); // Library using Wire and chip select pin D22

const byte cardCSPin = 24;

void setup() {
    Serial.begin(115200);

    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz
    SPI.begin();                        // SPI must be started first as well

    SD.begin(cardCSPin);                // SD library using chip select pin D24

    // Using memory allocation mode 80x60 8bpp and fahrenheit temperature mode
    flirController.init(LeptonFLiR_ImageStorageMode_80x60_8bpp, LeptonFLiR_TemperatureMode_Fahrenheit);

    // Setting use of AGC for histogram equalization (since we only have 8-bit per pixel data anyways)
    flirController.agc_setAGCEnabled(ENABLED);

    flirController.sys_setTelemetryEnabled(ENABLED); // Ensure telemetry is enabled

    SD.rmdir("FLIR");                   // Starting fresh with new frame captures
}

uint32_t lastFrameNumber = -1;          // Tracks for when a new frame is available

void loop() {
    if (flirController.readNextFrame()) { // Read next frame and store result into internal imageData
        uint32_t frameNumber = flirController.getTelemetryFrameCounter();

        if (frameNumber > lastFrameNumber) { // Frame counter increments every 3rd frame due to export restrictions
            lastFrameNumber = frameNumber;

            char fileName[] = "FLIR/IMG0000.BMP";
            uint16_t fileNumber = (uint16_t)(frameNumber / 3);
            wordsToHexString((uint16_t *)&fileNumber, 1, &fileName[8], 4);

            File bmpFile = SD.open(fileName, FILE_WRITE);

            if (bmpFile) {
                writeBMPFile(bmpFile,
                             flirController.getImageData(),
                             flirController.getImageWidth(),
                             flirController.getImageHeight(),
                             flirController.getImagePitch());

                bmpFile.close();

                Serial.print(fileName);
                Serial.println(" written...");
            }
        }

        // Occasionally flat field correction normalization needs ran
        if (flirController.getShouldRunFFCNormalization())
            flirController.sys_runFFCNormalization();
    }
}

// Writes a BMP file out, code from: http://stackoverflow.com/questions/2654480/writing-bmp-image-in-pure-c-c-without-other-libraries
void writeBMPFile(File &bmpFile, byte *imageData, int width, int height, int pitch) {
    byte file[14] = {
        'B','M', // magic
        0,0,0,0, // size in bytes
        0,0, // app data
        0,0, // app data
        40+14,0,0,0 // start of data offset
    };
    byte info[40] = {
        40,0,0,0, // info hd size
        0,0,0,0, // width
        0,0,0,0, // heigth
        1,0, // number color planes
        24,0, // bits per pixel
        0,0,0,0, // compression is none
        0,0,0,0, // image bits size
        0x13,0x0B,0,0, // horz resoluition in pixel / m
        0x13,0x0B,0,0, // vert resolutions (0x03C3 = 96 dpi, 0x0B13 = 72 dpi)
        0,0,0,0, // #colors in pallete
        0,0,0,0, // #important colors
    };

    uint32_t padSize  = (4-(width*3)%4)%4;
    uint32_t sizeData = width*height*3 + height*padSize;
    uint32_t sizeAll  = sizeData + sizeof(file) + sizeof(info);

    file[ 2] = (byte)((sizeAll      ) & 0xFF);
    file[ 3] = (byte)((sizeAll >>  8) & 0xFF);
    file[ 4] = (byte)((sizeAll >> 16) & 0xFF);
    file[ 5] = (byte)((sizeAll >> 24) & 0xFF);
    info[ 4] = (byte)((width      ) & 0xFF);
    info[ 5] = (byte)((width >>  8) & 0xFF);
    info[ 6] = (byte)((width >> 16) & 0xFF);
    info[ 7] = (byte)((width >> 24) & 0xFF);
    info[ 8] = (byte)((height      ) & 0xFF);
    info[ 9] = (byte)((height >>  8) & 0xFF);
    info[10] = (byte)((height >> 16) & 0xFF);
    info[11] = (byte)((height >> 24) & 0xFF);
    info[20] = (byte)((sizeData      ) & 0xFF);
    info[21] = (byte)((sizeData >>  8) & 0xFF);
    info[22] = (byte)((sizeData >> 16) & 0xFF);
    info[23] = (byte)((sizeData >> 24) & 0xFF);

    bmpFile.write((byte *)file, sizeof(file));
    bmpFile.write((byte *)info, sizeof(info));

    byte pad[3] = {0,0,0};
    imageData += (height - 1) * pitch;

    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            byte pixel[3]; // blue green red
            pixel[0] = pixel[1] = pixel[2] = imageData[x];

            bmpFile.write((byte *)pixel, 3);
        }

        bmpFile.write((byte *)pad, padSize);
        imageData -= pitch;
    }
}

```

### Software I2C Example

In this example, we utilize the software I2C functionality for chips that do not have a hardware I2C bus.

If one uncomments the line below inside the main header file (or defines it via custom build flag), software I2C mode for the library will be enabled.

In LeptonFLiR.h:
```Arduino
// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
#define LEPFLIR_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary
```

In main sketch:
```Arduino
#include "LeptonFLiR.h"

#define SCL_PIN 2                       // Setup defines are written before library include
#define SCL_PORT PORTD 
#define SDA_PIN 0 
#define SDA_PORT PORTC 

#if F_CPU >= 16000000
#define I2C_FASTMODE 1                  // Running a 16MHz processor allows us to use I2C fast mode
#endif

#include "SoftI2CMaster.h"              // Include must come after setup defines

const byte csPin = 4;
LeptonFLiR flirController(csPin);       // Library using chip select pin 4

void setup() {
    Serial.begin(115200);

    i2c_init();                         // Software I2C must be started first
    SPI.begin();                        // SPI must be started first as well

    // Using lowest memory allocation mode 20x15 8bpp and default celsius temperature mode
    flirController.init(LeptonFLiR_ImageStorageMode_20x15_8bpp);

    flirController.sys_setTelemetryEnabled(DISABLED); // Default mode is enabled
}

void loop() {
    flirController.readNextFrame();     // Reads next frame and stores result into internal imageData
}

```

## Module Info

In this example, we enable debug output support.

If one uncomments the line below inside the main header file (or defines it via custom build flag), debug output support will be enabled and the printModuleInfo() method will become available. Calling this method will display information about the module itself, including initalized states, register values, current settings, etc. Additionally, all library calls being made will display internal debug information about the structure of the call itself. An example of this output is shown below.

In LeptonFLiR.h:
```Arduino
// Uncomment this define to enable debug output.
#define LEPFLIR_ENABLE_DEBUG_OUTPUT     1
```

In main sketch:
```Arduino
#include "LeptonFLiR.h"

LeptonFLiR flirController;

void setup() {
    Serial.begin(115200);

    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz
    SPI.begin();                        // SPI must be started first as well

    // Using 40x30 8bpp memory allocation mode and default celsius temperature mode
    flirController.init(LeptonFLiR_ImageStorageMode_40x30_8bpp);

    flirController.printModuleInfo();
}

```

In serial monitor:
```
 ~~~ LeptonFLiR Module Info ~~~

Chip Select Pin:
D34 (active-low)

SPI Port Speed:
10.50MHz (SPI_CLOCK_DIV8)

Image Storage Mode:
1: LeptonFLiR_ImageStorageMode_80x60_8bpp

Temperature Mode:
1: LeptonFLiR_TemperatureMode_Fahrenheit

Memory Footprint:
Image Data: 4815B, SPI Frame Data: 191B, Telemetry Data: 164B, Total: 5170B

Power Register:
    LeptonFLiR::readRegister regAddress: 0x0
      LeptonFLiR::readRegister retVal: 0x0
0x0

Status Register:
    LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
0x6

AGC Enabled:
LeptonFLiR::agc_getAGCEnabled
  LeptonFLiR::receiveCommand cmdCode: 0x100
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x100
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[2]: 0x1-0x0
enabled

AGC Policy:
LeptonFLiR::agc_getAGCPolicy
  LeptonFLiR::receiveCommand cmdCode: 0x104
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x104
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[2]: 0x1-0x0
1: LEP_AGC_HEQ

AGC HEQ Scale Factor:
LeptonFLiR::agc_getHEQScaleFactor
  LeptonFLiR::receiveCommand cmdCode: 0x144
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x144
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[2]: 0x0-0x0
0: LEP_AGC_SCALE_TO_8_BITS

AGC Calculation Enabled:
LeptonFLiR::agc_getAGCCalcEnabled
  LeptonFLiR::receiveCommand cmdCode: 0x148
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x148
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[2]: 0x1-0x0
enabled

SYS Camera Status:
LeptonFLiR::sys_getCameraStatus
  LeptonFLiR::receiveCommand cmdCode: 0x204
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x204
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[4]: 0x0-0x0-0x0-0x0
0: LEP_SYSTEM_READY

FLiR Serial Number:
LeptonFLiR::sys_getFlirSerialNumber
  LeptonFLiR::receiveCommand cmdCode: 0x208
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x208
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[4]: 0x1111-0x2222-0x3333-0x4444
1111:2222:3333:4444

Customer Serial Number:
LeptonFLiR::sys_getCustomerSerialNumber
  LeptonFLiR::receiveCommand cmdCode: 0x228
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x228
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[16]: 0x1111-0x2222-0x3333-0x4444-0x5555-0x6666-0x7777-0x8888-0x9999-0xAAAA-0xBBBB-0xCCCC-0xDDDD-0xEEEE-0xFFFF-0xABCD
1111:2222:3333:4444:5555:6666:7777:8888:9999:AAAA:BBBB:CCCC:DDDD:EEEE:FFFF:ABCD

Camera Uptime:
LeptonFLiR::sys_getCameraUptime
  LeptonFLiR::receiveCommand cmdCode: 0x20C
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x20C
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[2]: 0x6FE4-0x64
6582244 ms

Sys Aux Temperature:
LeptonFLiR::sys_getAuxTemperature
  LeptonFLiR::receiveCommand cmdCode: 0x210
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x210
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[1]: 0x778B
91.18�F

Sys FPA Temperature:
LeptonFLiR::sys_getFPATemperature
  LeptonFLiR::receiveCommand cmdCode: 0x214
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x214
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[1]: 0x788D
95.83�F

Telemetry Enabled:
LeptonFLiR::sys_getTelemetryEnabled
  LeptonFLiR::receiveCommand cmdCode: 0x218
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x218
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[2]: 0x1-0x0
enabled

Vid Polarity:
LeptonFLiR::vid_getPolarity
  LeptonFLiR::receiveCommand cmdCode: 0x300
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x300
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[2]: 0x0-0x0
0: LEP_VID_WHITE_HOT

Vid Pseudo Color Lookup Table:
LeptonFLiR::vid_getPseudoColorLUT
  LeptonFLiR::receiveCommand cmdCode: 0x304
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x304
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[2]: 0x1-0x0
1: LEP_VID_FUSION_LUT

Vid Focus Calculation Enabled:
LeptonFLiR::vid_getFocusCalcEnabled
  LeptonFLiR::receiveCommand cmdCode: 0x30C
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x30C
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[2]: 0x0-0x0
disabled

Vid Freeze Enabled:
LeptonFLiR::vid_getFreezeEnabled
  LeptonFLiR::receiveCommand cmdCode: 0x324
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, value: 0x324
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister retVal: 0x6
      LeptonFLiR::readDataRegister readWords[2]: 0x0-0x0
disabled
```
