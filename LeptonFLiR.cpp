/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016 NachtRaveVL      <nachtravevl@gmail.com>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    This permission notice shall be included in all copies or
    substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.

    Lepton-FLiR-Arduino - Version 0.1
*/

#include "LeptonFLiR.h"

LeptonFLiR::LeptonFLiR(TwoWire& wire, byte csEnablePin) {
    _wire = &wire;
    _csEnablePin = csEnablePin;
}

void LeptonFLiR::init() {
    pinMode(_csEnablePin, OUTPUT);
    digitalWrite(_csEnablePin, HIGH);
}


// Old stuff beyond this point

const byte flirCSEnablePin = 49;
word flirImage[60][80];

void flirLeptonCommand(word commandID, byte commandType) {
    byte error;
    Wire.beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    Wire.write(0x00);
    Wire.write(0x04);

    if (commandID & LEP_OEM_MODULE_BASE == LEP_OEM_MODULE_BASE)
        commandID = 0x4800 | (commandID & 0x00ff);

    Wire.write((commandID >> 8) & 0xff);
    Wire.write(((commandID & 0xfc) | (commandType & 0x03)) & 0xff);

    error = Wire.endTransmission();
    if (error != 0) {
        Serial.print("error=");
        Serial.println(error);
    }
}

void flirEnableAGC() {
    byte error;
    Wire.beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    Wire.write(0x01);
    Wire.write(0x05);
    Wire.write(0x00);
    Wire.write(0x01);

    error = Wire.endTransmission();
    if (error != 0) {
        Serial.print("error=");
        Serial.println(error);
    }
}

void flirSetReg(word reg) {
    byte error;
    Wire.beginTransmission(LEP_I2C_DEVICE_ADDRESS);
    Wire.write((reg >> 8) & 0xff);
    Wire.write(reg & 0xff);

    error = Wire.endTransmission();
    if (error != 0) {
        Serial.print("error=");
        Serial.println(error);
    }
}

//Status reg 15:8 Error Code  7:3 Reserved 2:Boot Status 1:Boot Mode 0:busy
word flirReadReg(word reg) {
    word reading;
    flirSetReg(reg);

    Wire.requestFrom(LEP_I2C_DEVICE_ADDRESS, 2);

    reading = Wire.read() << 8;
    reading |= Wire.read();

    Serial.print("reg:");
    Serial.print(reg);
    Serial.print("==0x");
    Serial.print(reading, HEX);
    Serial.print(" binary:");
    Serial.println(reading, BIN);
    return reading;
}

int flirReadData() {
    word data;
    word payloadLength;

    while (flirReadReg(0x2) & 0x01) {
        Serial.println("busy");
    }

    payloadLength = flirReadReg(0x6);
    Serial.print("payloadLength=");
    Serial.println(payloadLength);

    Wire.requestFrom(LEP_I2C_DEVICE_ADDRESS, payloadLength);
    //flirSetReg(0x08); // TODO: WTF is this even doing? Why is it here? It came from GitHub.
    for (int i = 0; i < (payloadLength / 2); ++i) {
        data = Wire.read() << 8;
        data |= Wire.read();
        Serial.println(data, HEX);
    }
}

void demoFlirReading() {
    while (flirReadReg(0x2) & 0x01) {
        Serial.println("busy");
    }

    Serial.println("SYS Camera Status");
    flirLeptonCommand(LEP_CID_SYS_CAM_STATUS, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS Customer Serial Number");
    flirLeptonCommand(LEP_CID_SYS_CUST_SERIAL_NUMBER, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS FLiR Serial Number");
    flirLeptonCommand(LEP_CID_SYS_FLIR_SERIAL_NUMBER, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS Camera Uptime");
    flirLeptonCommand(LEP_CID_SYS_CAM_UPTIME, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS Aux Temperature Kelvin");
    flirLeptonCommand(LEP_CID_SYS_AUX_TEMPERATURE_KELVIN, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS Fpa Temperature Kelvin");
    flirLeptonCommand(LEP_CID_SYS_FPA_TEMPERATURE_KELVIN, FLIR_CMD_GET);
    flirReadData();

    Serial.println("AGC Enable State");
    flirLeptonCommand(LEP_CID_AGC_ENABLE_STATE, FLIR_CMD_GET);
    flirReadData();

    Serial.println("SYS Telemetry Enable State");
    flirLeptonCommand(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, FLIR_CMD_GET);
    flirReadData();

    /*Serial.println("OEM Chip Mask Revision");
    flirLeptonCommand(LEP_CID_OEM_CHIP_MASK_REVISION, FLIR_CMD_GET);
    flirReadData();

    Serial.println("OEM Camera Software Revision");
    flirLeptonCommand(LEP_CID_OEM_CAM_SOFTWARE_REVISION, FLIR_CMD_GET);
    flirReadData();*/
}

void flirReadLeptonFrame(int fillRow) {
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));

    if (fillRow >= 0 && fillRow < 60) {
        byte packetID;
        byte packetRow;
        word packetCRC;

        //REG_PIOB_ODSR &= ~0x01;
        digitalWrite(flirCSEnablePin, LOW);

        do {
            packetID = SPI.transfer(0x00);
            packetRow = SPI.transfer(0x00);
            packetCRC = SPI.transfer(0x00) << 8;
            packetCRC |= SPI.transfer(0x00);

            if (packetID & 0x0f == 0x0f)
                delay(200); // spec says 185, doing 200 to ensure timeout
        } while (packetID & 0x0f == 0x0f);

        // CRC check polynominal: x^16 + x^12 + x^5 + x^0
        // The CRC is calculated over the entire packet, including the ID and CRC fields.
        // However, the four most-significant bits of the ID and all sixteen bits of the
        // CRC are set to zero for calculation of the CRC. There is no requirement for the
        // host to verify the CRC. However, if the host does find a CRC mismatch, it is
        // recommended to re-synchronize the VoSPI stream to prevent potential misalignment.

        Serial.print("{ packetID=");
        Serial.print(packetID, HEX);
        Serial.print(", packetRow=");
        Serial.print(packetRow, HEX);
        Serial.print(", packetCRC=");
        Serial.print(packetCRC, HEX);
        Serial.println(" }");

        for (int i = 0; i < 80; ++i) {
            flirImage[fillRow][i] = SPI.transfer(0x00) << 8;
            flirImage[fillRow][i] |= SPI.transfer(0x00);
        }

        digitalWrite(flirCSEnablePin, HIGH);
        //REG_PIOB_ODSR |= 0x01;
    }

    SPI.endTransaction();
}
