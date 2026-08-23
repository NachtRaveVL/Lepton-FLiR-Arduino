// Lepton-FLiR-Arduino Advanced Example
// In this example, we will utilize various features of the library. We will be using
// Wire1, which is only available on boards with SDA1/SCL1 (e.g. Due/Teensy/etc.) - change
// to Wire if Wire1 is unavailable.

#include "LeptonFLiR.h"

const byte flirCSPin = 22;
LeptonFLiR flirController(Wire1, 400000, flirCSPin); // Library using chip select pin D22, and Wire1 @400kHz

void setup() {
    Serial.begin(115200);               // Begin Serial, SPI, and Wire interfaces
#ifdef __SAM3X8E__
    // Arduino Due has SPI library that manages the CS pin for us
    SPI.begin(flirController.getChipSelectPin());
#else
    SPI.begin();
#endif
    Wire1.begin();
    Wire1.setClock(flirController.getI2CSpeed());

    // Initializes module using Lepton v1 camera, and default celsius temperature mode
    // NOTE: Make sure to change this to what hardware camera version you're using! (see manufacturer website)
    flirController.init(LeptonFLiR_CameraType_Lepton1);

    flirController.sys_setTelemetryEnabled(ENABLED); // Ensure telemetry is enabled
}

void loop() {
    if (flirController.tryReadNextFrame()) { // Establishes sync, then reads next frame into raw data buffer
        // Frame settings describe the frame that was just captured, not the camera's live configuration
        Serial.print("Frame #");
        Serial.print(flirController.getFrameNumber());
        Serial.print(" [");
        Serial.print(flirController.getImageWidth());
        Serial.print("x");
        Serial.print(flirController.getImageHeight());
        Serial.print(", ");
        Serial.print(flirController.getImageBpp());
        Serial.print(" bytes/pixel raw] center pixel: ");

        // Individual pixels are accessed through getImagePixelData(), which handles VoSPI packet layout
        const int centerY = flirController.getImageHeight() / 2;
        const int centerX = flirController.getImageWidth() / 2;
        LeptonFLiR_PixelData pixel = flirController.getImagePixelData(centerY, centerX);

        if (flirController.getPseudoColorLUTEnabled()) {
            Serial.print("RGB(");
            Serial.print(pixel.pclut.red);
            Serial.print(",");
            Serial.print(pixel.pclut.green);
            Serial.print(",");
            Serial.print(pixel.pclut.blue);
            Serial.print(")");
        }
        else if (flirController.getAGCEnabled())
            Serial.print(pixel.agc.value);
        else if (flirController.getTLinearEnabled()) {
            Serial.print(pixel.tlinear.value);
            Serial.print(" TLinear");
        }
        else
            Serial.print(pixel.std.value);

        // getImageOutputData() provides the complete processed image as a normal row-oriented buffer
        byte *imageData = flirController.getImageOutputData();
        if (imageData) {
            Serial.print(", output: ");
            Serial.print(flirController.getImageOutputTotalSize());
            Serial.print(" bytes @ pitch ");
            Serial.print(flirController.getImageOutputPitch());
        }

        // Telemetry can be accessed either through individual getters or as a processed structure
        LeptonFLiR_TelemetryData *telemetry = flirController.getTelemetryOutputData();
        if (telemetry) {
            Serial.print(", telemetry frame: ");
            Serial.print(telemetry->frameCounter);
        }

        Serial.println();
    }
}
