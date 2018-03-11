// i2c.h -- I2C Setup STM32F103

#ifndef I2C_H
#define I2C_H
#include "stm32f103xb.h"    // Need as direct reference to HW

#include "buffer_class.h"

// Define function return status
typedef enum {Success = 0, Error = !Success} Status;

class I2C {
    private:    // Can only be accessed by fuctions in this class.
        Buffer& buffer;
    public:     // Can be accessed by any function in program.
        // Default Constructor
        I2C(Buffer& rx_buffer):buffer(rx_buffer)
        {
            // Anything which needs to be executed when class instansiated.
        }
        //uint8_t buffer;
        
        Status I2C1Setup();

        Status start(uint8_t SlaveAddr);
        Status write(uint8_t Data);
        Status stop();
        Status readbyte(uint8_t SlaveAddr);    // Read 1 Byte
        Status read2bytes(uint8_t SlaveAddr);  // Read 2 Bytes
        Status read3bytes(uint8_t SlaveAddr, uint8_t NumberBytesToRead);  // Read 3+ Bytes
        Status read(uint8_t NumberBytesToRead, uint8_t slaveAddress);
        uint8_t getbyte(); // Read a Byte from the i2c_rx_buffer

};

#endif // I2C_H
