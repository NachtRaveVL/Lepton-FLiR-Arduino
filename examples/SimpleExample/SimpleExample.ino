// Lepton-FLiR-Arduino Simple Example

#include "LeptonFLiR.h"

LeptonFLiR flirController;              // Library using default chip select pin D10, and default Wire @400kHz

void setup() {
    Serial.begin(115200);               // Library will begin Wire/SPI, so we just need to begin Serial

    // Initializes module using Lepton v1 camera, default celsius temperature mode, and also begins Wire/SPI
    // NOTE: Make sure to change this to what hardware camera version you're using! (see manufacturer website)
    flirController.init(LeptonFLiR_CameraType_Lepton1);
}

void loop() {
    flirController.tryReadNextFrame();  // Establishes sync, then reads next frame into raw data buffer
}
