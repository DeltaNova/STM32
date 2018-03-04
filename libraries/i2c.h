// i2c.h -- I2C Setup STM32F103

#ifndef I2C_H
#define I2C_H

typedef enum {Success = 0, Error = !Success} Status;

class I2C {
    private:    // Can only be accessed by fuctions in this class.
    public:     // Can be accessed by any function in program.
        I2C(){  // Default Constructor
            // Anything which needs to be executed when class instansiated.
        }
        
        // Define function return status
        

        Status I2C1Setup();

        Status start(uint8_t SlaveAddr);
        Status I2CWriteData(uint8_t Data);
        Status stop();
        Status I2CReadByte(uint8_t SlaveAddr, volatile struct Buffer *i2c_rx_buffer);    // Read 1 Byte
        Status I2CRead2Bytes(uint8_t SlaveAddr, volatile struct Buffer *i2c_rx_buffer);  // Read 2 Bytes
        Status I2CRead3Bytes(uint8_t SlaveAddr, uint8_t NumberBytesToRead, volatile struct Buffer *i2c_rx_buffer);  // Read 3+ Bytes
        Status I2CReadData(uint8_t NumberBytesToRead, uint8_t slaveAddress, volatile struct Buffer *i2c_rx_buffer);
        void I2CRead(uint8_t NumberBytesToRead, uint8_t slaveAddress);

};
#endif // I2C_H
