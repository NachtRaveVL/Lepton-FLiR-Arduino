// Lepton-FLiR-Arduino Simple Example

#include "LeptonFLiR.h"

LeptonFLiR flirController;              // Library using default Wire and default chip select pin D53

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
