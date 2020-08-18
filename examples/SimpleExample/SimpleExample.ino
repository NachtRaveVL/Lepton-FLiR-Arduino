// Lepton-FLiR-Arduino Simple Example

#include "LeptonFLiR.h"

LeptonFLiR flirController;              // Library using default Wire and default chip select pin D10

void setup() {
    Serial.begin(115200);

    Wire.begin();                       // Wire must be started
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    // SPI must also be started
#ifdef __SAM3X8E__
    // Arduino Due has SPI library that manages the CS pin for us.
    SPI.begin(flirController.getChipSelectPin()) // Get defaulted CS pin from library
#else
    SPI.begin();
#endif

    // Using Lepton v1 camera and default celsius temperature mode
    // NOTE: Make sure to change this to what camera version you're using.
    flirController.init(LeptonFLiR_CameraType_Lepton1);
}

void loop() {
    flirController.tryReadNextFrame();  // Establishes sync, then reads next frame into raw data buffer
}
