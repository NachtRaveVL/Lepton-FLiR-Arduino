// Lepton-FLiR-Arduino Module Info
// In this example, we enable debug output support. If one uncomments the line below
// inside the main header file (or defines it via custom build flag), debug output
// support will be enabled and the printModuleInfo() method will become available.
// Calling this method will display information about the module itself, including
// initalized states, register values, current settings, etc. Additionally, all
// library calls being made will display internal debug information about the
// structure of the call itself.
//
// In LeptonFLiR.h:
// // Uncomment this define to enable debug output.
// #define LEPFLIR_ENABLE_DEBUG_OUTPUT     1

#include "LeptonFLiR.h"

LeptonFLiR flirController;

void setup() {
    Serial.begin(115200);

    Wire.begin();                       // Wire must be started
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    // SPI must also be started
#ifdef __SAM3X8E__
    // Arduino Due has SPI library that manages the CS pin for us.
    SPI.begin(flirController.getChipSelectPin())
#else
    SPI.begin();
#endif

    // Using Lepton v1 camera and default celsius temperature mode
    // NOTE: Make sure to change this to what camera version you're using.
    flirController.init(LeptonFLiR_CameraType_Lepton1);

    flirController.printModuleInfo();
}

void loop() {
}
