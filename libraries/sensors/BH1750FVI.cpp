// BH1750FVI.cpp - I2C Lux Sensor
#include "BH1750FVI.h"

void BH1750FVI::setup(){
    // Setup Function for the BH1750FVI Lux Sensor
    i2c.start(Address);        // Slave Address
    i2c.write(0x01);            // BH1750FVI - Power On
    i2c.stop();                 // Required as part of BH1750FVI I2C Comms

    i2c.start(Address);        // Slave Address
    //i2c.write(0x20);          // BH1750FVI One Time H-Res Mode.
    i2c.write(0x13);            // BH1750FVI Continuous Mode
    i2c.stop();                 // Required as part of BH1750FVI I2C Comms 
}

void BH1750FVI::read(){
    // Read the Sensor
    // Reads 2 Byte Measurement into i2c_rx_buffer
    i2c.read(2, Address);                      // Read 2 Bytes from sensor
    HighByte = i2c.getbyte();                   // High Byte
    LowByte = i2c.getbyte();                    // Low Byte
    LuxBytes = (HighByte <<8) + LowByte;        // Combine Bytes
}

uint8_t BH1750FVI::getLowByte(){
    return LowByte;
}

uint8_t BH1750FVI::getHighByte(){
    return HighByte;
}

uint16_t BH1750FVI::getLuxByte(){
    return LuxBytes;
}

uint16_t BH1750FVI::getLuxValue(){
    // Convert LuxBytes into LuxValue - H-Resolution Mode
    // Default Settings in H-Resolution mode = Resolution of 0.83lx/count.
    // Therefore count/1.2 gives lux value
    uint16_t LuxValue = LuxBytes / 1.2;
    return LuxValue;
}
