// Lepton-FLiR-Arduino Advanced Example
// In this example, we will utilize various features of the library. We will be using
// Wire1, which is only available on boards with SDA1/SCL1 (Due, Zero, etc.) - change to
// Wire if Wire1 is unavailable. We will also be using the digitalWriteFast library,
// available at: https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

#include "LeptonFLiR.h"
#include "digitalWriteFast.h"

const byte flirCSPin = 22;
LeptonFLiR flirController(Wire1, flirCSPin); // Library using Wire1 and chip select pin D22

// Fast CS enable/disable routines, using the digitalWriteFast library
static void fastEnableCS(byte pin) { digitalWriteFast(pin, LOW); }
static void fastDisableCS(byte pin) { digitalWriteFast(pin, HIGH); }

void setup() {
    Serial.begin(115200);

    Wire1.begin();                      // Wire1 must be started
    Wire1.setClock(400000);             // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    // SPI must also be started
#ifdef __SAM3X8E__
    // Arduino Due has SPI library that manages the CS pin for us.
    SPI.begin(flirCSPin)
#else
    SPI.begin();
#endif

    // Using Lepton v1 camera and default celsius temperature mode
    // NOTE: Make sure to change this to what camera version you're using.
    flirController.init(LeptonFLiR_CameraType_Lepton1);

    // Setting use of fast enable/disable methods for chip select
    flirController.setFastCSFuncs(fastEnableCS, fastDisableCS);

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
