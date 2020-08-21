// Lepton-FLiR-Arduino Module Info
// In this example, we enable debug output support to print out module diagnostic
// information. If one uncomments the line below inside the main header file (or defines
// it via custom build flag), debug output support will be enabled and the
// printModuleInfo() method will become available. Calling this method will display
// information about the module itself, including initalized states, register values,
// current settings, etc. Additionally, all library calls being made will display
// internal debug information about the structure of the call itself.
//
// In LeptonFLiR.h:
// // Uncomment this define to enable debug output.
// #define LEPFLIR_ENABLE_DEBUG_OUTPUT

#include "LeptonFLiR.h"

LeptonFLiR flirController;              // Library using default chip select pin D10, and default Wire @400kHz

void setup() {
    Serial.begin(115200);               // Library will begin Wire/SPI, so we just need to begin Serial

    // Initializes module using Lepton v1 camera, default celsius temperature mode, and also begins Wire/SPI
    // NOTE: Make sure to change this to what hardware camera version you're using! (see manufacturer website)
    flirController.init(LeptonFLiR_CameraType_Lepton1);

    flirController.printModuleInfo();   // Prints module diagnostic information
}

void loop() {
}
