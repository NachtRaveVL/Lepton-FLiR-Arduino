# Lepton-FLiR-Arduino
Arduino Library for the Lepton FLiR Thermal Camera Module.

**Lepton-FLiR-Arduino v0.9.8**

Library to control a Lepton FLiR (forward looking infrared) thermal camera module from an Arduino board (Due, Zero, etc. recommended).  
Licensed under the non-restrictive MIT license.

Created by NachtRaveVL, August 1st, 2016.

This library allows communication with boards running a Lepton FLiR thermal camera module. It provides a wide range of functionality from adjustable memory footprint size, adjustable temperature display mode, fast chip select enable/disable routines, to exposing the full functionality of the thermal camera itself.

Dependencies include Scheduler if on a ARM/ARMD architecture (Due, Zero, etc.), but usage can be disabled via library setup defines.

Parts of this library are derived from the Lepton FLiR software development SDK, Copyright 2011,2012,2013,2014 FLIR Systems - Commercial Vision Systems.

## Library Setup

There are several defines inside of the library's header file that allows for more fine-tuned control.

```Arduino
// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
//#define LEPFLIR_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define to disable usage of the Scheduler library on SAM/SAMD architecures.
//#define LEPFLIR_DISABLE_SCHEDULER       1   // https://github.com/arduino-libraries/Scheduler

// Uncomment this define to disable 16 byte aligned memory allocations (may hinder performance).
//#define LEPFLIR_DISABLE_ALIGNED_MALLOC  1

// Uncomment this define if wanting to exclude extended i2c functions from compilation.
//#define LEPFLIR_EXCLUDE_EXT_I2C_FUNCS   1

// Uncomment this define to enable debug output.
//#define LEPFLIR_ENABLE_DEBUG_OUTPUT     1
```

## Hookup Instructions

Make sure to hookup the module's SPI lines MISO, MOSI, CLK (aka SCK), and CS (aka SS) correctly (Due, Zero, ATmega, etc. often use pins 50=MISO, 51=MOSI, 52=SCK, 53=SS, but one can just simply use the ICSP header pins ICSP-1=MISO, ICSP-4=MOSI, ICSP-3=SCK, which are consistent across all boards - Due boards also have a SPI header, which is set up exactly like the ICSP header). The module's MOSI line can simply be grounded since the module only uses SPI for slave-out data transfers (slave-in data transfers being ignored). The SS pin may be any digital output pin, with usage being active-low. The recommended VCC power supply and logic level is 3.3v, but 5v is also supported. The two issolated power pins on the side of the module's breakout can safely be left disconnected. The minimum SPI transfer rate is ~2.2MHz, which means one needs at least an 8MHz processor, but a 16MHz processor is the recommended minimum given the actual processing work involved to resize/BLIT the final image. The actual SPI transfer rate selected will be the first rate equal to or below 20MHz given the SPI clock divider (i.e. processor speed /2, /4, /8, /16, ..., /128).

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

## Example Usage

Below are several examples of library usage.

### Simple Example
```Arduino
#include "LeptonFLiR.h"

LeptonFLiR flirController();            // Library using default Wire and default chip select pin D53

void setup() {
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

In this example, we will utilize various features of the library. We will be using Wire1, which is only available on boards with SDA1/SCL1 (Due, Zero, etc.) - change to Wire if Wire1 is unavailable. We will also be using the digitalWriteFast library, available at: https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

```Arduino
#include "LeptonFLiR.h"
#include "digitalWriteFast.h"

const byte flirCSPin = 22;
LeptonFLiR flirController(Wire1, flirCSPin); // Library using Wire1 and chip select pin D22

// Fast CS enable/disable routines, using the digitalWriteFast library
static void fastEnableCS(byte pin) { digitalWriteFast(pin, LOW); }
static void fastDisableCS(byte pin) { digitalWriteFast(pin, HIGH); }

void setup() {
    Wire1.begin();                      // Wire1 must be started first
    Wire1.setClock(400000);             // Supported baud rates are 100kHz, 400kHz, and 1000kHz
    SPI.begin();                        // SPI must be started first as well

    // Using default memory allocation mode 80x60 16bpp and default celsius temperature mode
    flirController.init();

    // Setting use of fast enable/disable methods for chip select
    flirController.setFastCSFuncs(fastEnableCS, fastDisableCS);

    flirController.setTelemetryEnabled(ENABLED); // Ensure telemetry is enabled
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

In this example, we will copy out thermal image frames to individual BMP files located on a MicroSD card using the SD library. Note that you will need a MicroSD card reader module for this example to work. Both the FLiR module and MicroSD card reader module will be on the same SPI lines, just using different chip enable pins/wires.

```Arduino
#include "LeptonFLiR.h"
#include <SD.h>

const byte flirCSPin = 22;
LeptonFLiR flirController(Wire, flirCSPin); // Library using Wire and chip select pin D22

const byte cardCSPin = 24;

void setup() {
    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz
    SPI.begin();                        // SPI must be started first as well

    SD.begin(cardCSPin);                // SD library using chip select pin D24

    // Using memory allocation mode 80x60 8bpp and fahrenheit temperature mode
    flirController.init(LeptonFLiR_ImageStorageMode_80x60_8bpp, LeptonFLiR_TemperatureMode_Fahrenheit);

    // Setting use of AGC for histogram equalization (since we only have 8-bit per pixel data anyways)
    flirController.setAGCEnabled(ENABLED);

    flirController.setTelemetryEnabled(ENABLED); // Ensure telemetry is enabled

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
            flirController.runSysFFCNormalization();
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

In this example, we utilize the software I2C functionality for chips that do not have a hardware I2C bus. We must uncomment the LEPFLIR_ENABLE_SOFTWARE_I2C define in the libraries main header file for software I2C mode to be enabled.

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
    i2c_init();                         // Software I2C must be started first
    SPI.begin();                        // SPI must be started first as well

    // Using lowest memory allocation mode 20x15 8bpp and default celsius temperature mode
    flirController.init(LeptonFLiR_ImageStorageMode_20x15_8bpp);

    flirController.setTelemetryMode(DISABLED); // Default mode is enabled
}

void loop() {
    flirController.readNextFrame();     // Reads next frame and stores result into internal imageData
}

```

## Module Info

If one uncomments the LEPFLIR_ENABLE_DEBUG_OUTPUT define in the libraries main header file (thus enabling debug output) the printModuleInfo() method becomes available, which will display information about the module itself, including initalized states, register values, current settings, etc. All calls being made will display internal debug information about the structure of the call itself. An example of this output is shown here:

In LeptonFLiR.h:
```Arduino
// Uncomment this define to enable debug output.
#define LEPFLIR_ENABLE_DEBUG_OUTPUT     1
```

In main sketch:
```Arduino
LeptonFLiR flirController;

void setup() {
    // ...

    flirController.printModuleInfo();
}
```

In serial monitor:
```
 ~~~ LeptonFLiR Module Info ~~~

Chip Select Pin:
D53 (active-low)

SPI Port Speed:
10.50MHz (SPI_CLOCK_DIV8)

Image Storage Mode:
0: LeptonFLiR_ImageStorageMode_80x60_16bpp

Temperature Mode:
0: LeptonFLiR_TemperatureMode_Celsius

Memory Footprint:
Image Data: 9615B, SPI Frame Data: 191B, Telemetry Data: 164B, Total: 9970B

AGC Enabled:
LeptonFLiR::getAGCEnabled
  LeptonFLiR::receiveCommand cmdCode: 0x100
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x100-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x4
      LeptonFLiR::readRegister readWords[2]: 0x0-0x0
disabled

AGC Policy:
LeptonFLiR::getAGCPolicy
  LeptonFLiR::receiveCommand cmdCode: 0x104
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x104-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x4
      LeptonFLiR::readRegister readWords[2]: 0x1-0x0
1: LEP_AGC_HEQ

AGC HEQ Scale Factor:
LeptonFLiR::getAGCHEQScaleFactor
  LeptonFLiR::receiveCommand cmdCode: 0x144
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x144-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x4
      LeptonFLiR::readRegister readWords[2]: 0x0-0x0
0: LEP_AGC_SCALE_TO_8_BITS

AGC Calculation Enabled:
LeptonFLiR::getAGCCalcEnabled
  LeptonFLiR::receiveCommand cmdCode: 0x148
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x148-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x4
      LeptonFLiR::readRegister readWords[2]: 0x1-0x0
enabled

SYS Camera Status:
LeptonFLiR::getSysCameraStatus
  LeptonFLiR::receiveCommand cmdCode: 0x204
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x204-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x8
      LeptonFLiR::readRegister readWords[4]: 0x0-0x0-0x0-0x0
0: LEP_SYSTEM_READY

FLiR Serial Number:
LeptonFLiR::getSysFlirSerialNumber
  LeptonFLiR::receiveCommand cmdCode: 0x208
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x208-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x8
      LeptonFLiR::readRegister readWords[4]: 0x1-0x2-0x3-0x4
0001:0002:0003:0004

Customer Serial Number:
LeptonFLiR::getSysCustomerSerialNumber
  LeptonFLiR::receiveCommand cmdCode: 0x228
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x228-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x20
      LeptonFLiR::readRegister readWords[16]: 0x1-0x2-0x3-0x4-x05-0x6-0x7-0x8-0x9-0xA-0xB-0xC-0xD-0xE-0xF-0x10
0001:0002:0003:0004:0005:0006:0007:0008:0009:000A:000B:000C:000D:000E:000F:0010

Camera Uptime:
LeptonFLiR::getSysCameraUptime
  LeptonFLiR::receiveCommand cmdCode: 0x20C
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x20C-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x4
      LeptonFLiR::readRegister readWords[2]: 0x1B-0x0
27 ms

Sys Aux Temperature:
LeptonFLiR::getSysAuxTemperature
  LeptonFLiR::receiveCommand cmdCode: 0x210
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x210-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x7477
25.00°C

Sys FPA Temperature:
LeptonFLiR::getSysFPATemperature
  LeptonFLiR::receiveCommand cmdCode: 0x214
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x214-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x7477
25.00°C

Telemetry Enabled:
LeptonFLiR::getSysTelemetryEnabled
  LeptonFLiR::receiveCommand cmdCode: 0x218
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x218-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x4
      LeptonFLiR::readRegister readWords[2]: 0x1-0x0
enabled

Vid Polarity:
LeptonFLiR::getVidPolarity
  LeptonFLiR::receiveCommand cmdCode: 0x300
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x300-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x4
      LeptonFLiR::readRegister readWords[2]: 0x0-0x0
0: LEP_VID_WHITE_HOT

Vid Pseudo Color Lookup Table:
LeptonFLiR::getVidPseudoColorLUT
  LeptonFLiR::receiveCommand cmdCode: 0x304
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x304-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x4
      LeptonFLiR::readRegister readWords[2]: 0x1-0x0
1: LEP_VID_FUSION_LUT

Vid Focus Calculation Enabled:
LeptonFLiR::getVidFocusCalcEnabled
  LeptonFLiR::receiveCommand cmdCode: 0x30C
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x30C-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x4
      LeptonFLiR::readRegister readWords[2]: 0x0-0x0
disabled

Vid Freeze Enabled:
LeptonFLiR::getVidFreezeEnabled
  LeptonFLiR::receiveCommand cmdCode: 0x324
    LeptonFLiR::waitCommandBegin
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::writeRegister regAddress: 0x4, dataWords[2]: 0x324-0x0
    LeptonFLiR::waitCommandFinish
      LeptonFLiR::readRegister regAddress: 0x2
      LeptonFLiR::readRegister readWords[1]: 0x6
    LeptonFLiR::readRegister regAddress: 0x6
      LeptonFLiR::readRegister readWords[1]: 0x4
      LeptonFLiR::readRegister readWords[2]: 0x0-0x0
disabled
```
