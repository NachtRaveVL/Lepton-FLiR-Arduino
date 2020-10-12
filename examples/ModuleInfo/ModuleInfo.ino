// Lepton-FLiR-Arduino Module Info
// In this example, we enable debug output support to print out module diagnostic
// information. If one uncomments the line below inside the main header file (or defines
// it via custom build flag), debug output support will be enabled and the
// printModuleInfo() method will become available. Calling this method will display
// information about the module itself, including initalized states, register values,
// current settings, etc. Additionally, all library calls being made will display
// internal debug information about the structure of the call itself. You may refer to
// https://forum.arduino.cc/index.php?topic=602603.0 on how to define custom build flags
// manually via modifying platform[.local].txt.
//
// In LeptonFLiR.h:
// // Uncomment or -D this define to enable debug output.
// #define LEPFLIR_ENABLE_DEBUG_OUTPUT
//
// Alternatively, in platform[.local].txt:
// build.extra_flags=-DLEPFLIR_ENABLE_DEBUG_OUTPUT

#include "LeptonFLiR.h"

LeptonFLiR flirController;              // Library using default chip select pin D10, and default Wire @400kHz

void setup() {
    Serial.begin(115200);               // Begin Serial, SPI, and Wire interfaces
#ifdef __SAM3X8E__
    // Arduino Due has SPI library that manages the CS pin for us
    SPI.begin(flirController.getChipSelectPin());
#else
    SPI.begin();
#endif
    Wire.begin();

    // Initializes module using Lepton v1 camera, and default celsius temperature mode
    // NOTE: Make sure to change this to what hardware camera version you're using! (see manufacturer website)
    flirController.init(LeptonFLiR_CameraType_Lepton1);

    flirController.printModuleInfo();   // Prints module diagnostic information
}

void loop() {
}
