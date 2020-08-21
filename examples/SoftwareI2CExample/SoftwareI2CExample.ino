// Lepton-FLiR-Arduino Software I2C Example
// In this example, we utilize the software I2C functionality for chips that do not have
// a hardware I2C bus. If one uncomments the line below inside the main header file (or
// defines it via custom build flag), software I2C mode for the library will be enabled.
//
// In LeptonFLiR.h:
// // Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
// #define LEPFLIR_ENABLE_SOFTWARE_I2C             // http://playground.arduino.cc/Main/SoftwareI2CLibrary

#include "LeptonFLiR.h"

#define SCL_PIN 2                       // Setup defines for SoftI2CMaster are written before library include
#define SCL_PORT PORTD 
#define SDA_PIN 0 
#define SDA_PORT PORTC 

#if F_CPU >= 16000000
#define I2C_FASTMODE 1                  // Running a 16MHz processor allows us to use i2c fast mode
#endif

#include "SoftI2CMaster.h"              // Include must come after setup defines (see library setup)

const byte flirCSPin = 4;
LeptonFLiR flirController(flirCSPin);   // Library using chip select pin D4

void setup() {
    Serial.begin(115200);               // Library will begin Wire/SPI, so we just need to begin Serial

    // Initializes module using Lepton v1 camera, default celsius temperature mode, and also begins Wire/SPI
    // NOTE: Make sure to change this to what hardware camera version you're using! (see manufacturer website)
    flirController.init(LeptonFLiR_CameraType_Lepton1);

    flirController.sys_setTelemetryEnabled(DISABLED); // Default mode is enabled
}

void loop() {
    flirController.tryReadNextFrame();  // Establishes sync, then reads next frame into raw data buffer
}
