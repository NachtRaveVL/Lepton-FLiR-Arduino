#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

SerialMock Serial;
SPIClass SPI;
TwoWire Wire;
TwoWire Wire1;
int arduinoInterruptNumber = -1;
int arduinoInterruptMode = 0;
void (*arduinoInterruptHandler)() = nullptr;
