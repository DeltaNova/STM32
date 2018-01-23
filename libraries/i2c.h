// i2c.h -- I2C Setup STM32F103

#ifndef I2C_H
#define I2C_H

void I2CSetup(volatile struct Buffer *i2c_rx_buffer);
void I2CStart();
void I2CWriteMode(uint8_t SlaveAddr);
void I2CWriteData(uint8_t Data);
void I2CStop();
void I2CReadByte(uint8_t SlaveAddr, volatile struct Buffer *i2c_rx_buffer);    // Read 1 Byte
void I2CRead2Bytes(uint8_t SlaveAddr, volatile struct Buffer *i2c_rx_buffer);  // Read 2 Bytes
void I2CRead3Bytes(uint8_t SlaveAddr, uint8_t NumberBytesToRead, volatile struct Buffer *i2c_rx_buffer);  // Read 3+ Bytes
void I2CReadMode(uint8_t SlaveAddr);
void I2CReadData(uint8_t NumberBytesToRead, uint8_t slaveAddress);
void I2CRead(uint8_t NumberBytesToRead, uint8_t slaveAddress);

#endif // I2C_H
