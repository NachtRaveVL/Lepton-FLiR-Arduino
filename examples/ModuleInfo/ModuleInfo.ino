// Lepton-FLiR-Arduino Module Info
// If one uncomments the LEPFLIR_ENABLE_DEBUG_OUTPUT define in the libraries main header
// file (thus enabling debug output) the printModuleInfo() method becomes available,
// which will display information about the module itself, including initalized states,
// register values, current settings, etc. All calls being made will display internal
// debug information about the structure of the call itself.

// Uncomment this define to enable debug output.
#define LEPFLIR_ENABLE_DEBUG_OUTPUT     1

#include "LeptonFLiR.h"

LeptonFLiR flirController;

void setup() {
    Serial.begin(115200);

    flirController.printModuleInfo();
}

void loop() {
}
