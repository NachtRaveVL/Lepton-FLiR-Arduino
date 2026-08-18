// Lepton-FLiR-Arduino DMA Capture Example
// Uses DMA-backed asynchronous SPI when the board's SPI library supports it.
// On unsupported boards the DMA request is rejected and normal blocking SPI remains available.

#include "LeptonFLiR.h"

LeptonFLiR flirController;

void setup() {
    Serial.begin(115200);
    SPI.begin();
    Wire.begin();
    Wire.setClock(flirController.getI2CSpeed());

    flirController.init(LeptonFLiR_CameraType_Lepton3_5);

    if (flirController.setSPIDMAEnabled())
        Serial.println("DMA SPI capture enabled");
    else
        Serial.println("DMA SPI unavailable, using blocking SPI");
}

void loop() {
    if (flirController.tryReadNextFrame())
        Serial.println("Frame read success");
}
