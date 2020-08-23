// Lepton-FLiR-Arduino Advanced Example
// In this example, we will utilize various features of the library. We will be using
// Wire1, which is only available on boards with SDA1/SCL1 (e.g. Due/Mega/etc.) - change
// to Wire if Wire1 is unavailable. We will also be using the digitalWriteFast library,
// available at: https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

#include "LeptonFLiR.h"
#include "digitalWriteFast.h"

const byte flirCSPin = 22;
LeptonFLiR flirController(flirCSPin, Wire1); // Library using chip select pin D22, and Wire1 @400kHz

// Fast CS enable/disable routines, using the digitalWriteFast library
static void fastEnableCS(byte pin) { digitalWriteFast(pin, LOW); }
static void fastDisableCS(byte pin) { digitalWriteFast(pin, HIGH); }

void setup() {
    Serial.begin(115200);               // Library will begin Wire/SPI, so we just need to begin Serial

    // Setting use of fast enable/disable methods for chip select
    flirController.setFastCSFuncs(fastEnableCS, fastDisableCS);

    // Initializes module using Lepton v1 camera, default celsius temperature mode, and also begins Wire/SPI
    // NOTE: Make sure to change this to what hardware camera version you're using! (see manufacturer website)
    flirController.init(LeptonFLiR_CameraType_Lepton1);

    flirController.sys_setTelemetryEnabled(ENABLED); // Ensure telemetry is enabled
}

void loop() {
    if (flirController.tryReadNextFrame()) { // Establishes sync, then reads next frame into raw data buffer
        // Find the hottest spot on the frame
        int hotVal = 0; hotX, hotY;

        for (int y = 0; y < flirController.getImageHeight(); ++y) {
            for (int x = 0; x < flirController.getImageWidth(); ++x) {
                int val = flirController.getImageDataRowCol(y, x);

                if (val > hotVal) {
                    hotVal = val;
                    hotX = x; hotY = y;
                }
            }
        }

        Serial.print("Hottest point: [");
        Serial.print(hotX);
        Serial.print(",");
        Serial.print(hotY);
        Serial.println("]");
    }
}
