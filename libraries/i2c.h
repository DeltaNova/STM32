// i2c.h -- I2C Setup STM32F103

#ifndef I2C_H
#define I2C_H

// Define function return status
typedef enum {Success = 0, Error = !Success} Status;

class I2C {
    private:    // Can only be accessed by fuctions in this class.
    public:     // Can be accessed by any function in program.
        I2C(){  // Default Constructor
            // Anything which needs to be executed when class instansiated.
        }
        
        Status I2C1Setup();

        Status start(uint8_t SlaveAddr);
        Status write(uint8_t Data);
        Status stop();
        Status readbyte(uint8_t SlaveAddr, volatile struct Buffer *i2c_rx_buffer);    // Read 1 Byte
        Status read2bytes(uint8_t SlaveAddr, volatile struct Buffer *i2c_rx_buffer);  // Read 2 Bytes
        Status read3bytes(uint8_t SlaveAddr, uint8_t NumberBytesToRead, volatile struct Buffer *i2c_rx_buffer);  // Read 3+ Bytes
        Status read(uint8_t NumberBytesToRead, uint8_t slaveAddress, volatile struct Buffer *i2c_rx_buffer);

};
#endif // I2C_H
