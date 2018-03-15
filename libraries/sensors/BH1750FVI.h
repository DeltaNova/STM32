// BH1750FVI.h - I2C Lux Sensor
#include <stdint.h>         // Support for uint8_t
#include "i2c.h"            // Access to the I2C Class

#ifndef BH1750FVI_H
#define BH1750FVI_H
class BH1750FVI{
    private:
        I2C& i2c; // Holds i2c instance for use by class functions
        uint8_t HighByte;   // Holds Higher Byte of Lux Value
        uint8_t LowByte;    // Holds Lower Byte of Lux Value
        uint16_t LuxBytes;  // Holds the complete Lux Value
        uint8_t Address; // I2C Address of sensor
    public:
        // Require an reference to the I2C interface when creating instance.
        BH1750FVI(I2C& i2c, uint8_t ADDR)
        // Store the I2C reference in private.
        :i2c(i2c),Address(ADDR){} 
        // Setup function will use the i2c reference in private.
        void setup();   // Setup the Sensor
        void read();    // Read the Sensor
        uint8_t getLowByte();
        uint8_t getHighByte();
        uint16_t getLuxByte();  // Returns the value in LuxBytes
        // Converts LuxBytes into a Lux Value. Currently H-Res mode only.
        uint16_t getLuxValue(); 
};
#endif // BH1750FVI_H
