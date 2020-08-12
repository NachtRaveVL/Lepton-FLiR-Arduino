// Lepton-FLiR-Arduino Software I2C Example
// In this example, we utilize the software I2C functionality for chips that do not have
// a hardware I2C bus. If one uncomments the line below inside the main header file (or
// defines it via custom build flag), software I2C mode for the library will be enabled.
//
// In LeptonFLiR.h:
// // Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
// #define LEPFLIR_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

#include "LeptonFLiR.h"

#define SCL_PIN 2                       // Setup defines are written before library include
#define SCL_PORT PORTD 
#define SDA_PIN 0 
#define SDA_PORT PORTC 

#if F_CPU >= 16000000
#define I2C_FASTMODE 1                  // Running a 16MHz processor allows us to use I2C fast mode
#endif

#include "SoftI2CMaster.h"              // Include must come after setup defines

const byte csPin = 4;
LeptonFLiR flirController(csPin);       // Library using chip select pin 4

void setup() {
    Serial.begin(115200);

    i2c_init();                         // Software I2C must be started first
    SPI.begin();                        // SPI must be started first as well

    // Using lowest memory allocation mode 20x15 8bpp and default celsius temperature mode
    flirController.init(LeptonFLiR_ImageStorageMode_20x15_8bpp);

    flirController.sys_setTelemetryEnabled(DISABLED); // Default mode is enabled
}

void loop() {
    flirController.readNextFrame();     // Reads next frame and stores result into internal imageData
}
