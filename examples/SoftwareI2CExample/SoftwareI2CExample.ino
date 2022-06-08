// Lepton-FLiR-Arduino Software i2c Example
// In this example, we utilize a popular software i2c library for chips that do not have
// a hardware i2c bus. If one uncomments the line below inside the main header file (or
// defines it via custom build flag), software i2c mode for the library will be enabled.
// Additionally, you will need to correctly define SCL_PIN, SCL_PORT, SDA_PIN, and
// SDA_PORT according to your setup. I2C_FASTMODE=1 should be set for 16MHz+ processors.
// You may refer to https://forum.arduino.cc/index.php?topic=602603.0 on how to define
// custom build flags manually via modifying platform[.local].txt.
//
// In LeptonFLiR.h:
// // Uncomment or -D this define to enable usage of the software i2c library (min 4MHz+ processor).
// #define LEPFLIR_ENABLE_SOFTWARE_I2C             // http://playground.arduino.cc/Main/SoftwareI2CLibrary
//
// Alternatively, in platform[.local].txt:
// build.extra_flags=-DLEPFLIR_ENABLE_SOFTWARE_I2C

#include "LeptonFLiR.h"

// Setup defines for SoftI2CMaster are written before library include. That is because
// its header contains the full code definition, and should thus be included only once.
// The values for SCL_PORT and SDA_PORT are dependent upon which pins are used - refer to
// http://www.arduino.cc/en/Reference/PortManipulation to determine what you should use.
#define SCL_PIN 2
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
    Serial.begin(115200);               // Begin Serial, SPI, and SoftI2C interfaces
#ifdef __SAM3X8E__
    // Arduino Due has SPI library that manages the CS pin for us
    SPI.begin(flirController.getChipSelectPin());
#else
    SPI.begin();
#endif
    i2c_init();

    // Initializes module using Lepton v1 camera, and default celsius temperature mode
    // NOTE: Make sure to change this to what hardware camera version you're using! (see manufacturer website)
    flirController.init(LeptonFLiR_CameraType_Lepton1);

    flirController.sys_setTelemetryEnabled(DISABLED); // Default mode is enabled
}

void loop() {
    flirController.tryReadNextFrame();  // Establishes sync, then reads next frame into raw data buffer
}
