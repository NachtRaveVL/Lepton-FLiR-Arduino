// Lepton-FLiR-Arduino Module Info
// In this example, we enable debug output support. We copy the LEPFLIR_ENABLE_DEBUG_OUTPUT
// define from the libraries header file for debug output support to enable and the
// printModuleInfo() method to become available. Calling this method will display
// information about the module itself, including initalized states, register values,
// current settings, etc. All library calls being made will also display internal debug
// information about the structure of the call itself.

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
