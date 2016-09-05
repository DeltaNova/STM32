// serial.cpp

#include "serial.h"

void SerialSendByte(uint8_t data2send){
    // Working
    // Wait until TXE = 1 (indicates DR empty)
    while(!(USART1->SR & 0x00000080));
    // Put data2send into Data Register
    USART1->DR = data2send;
    return;
}

uint8_t SerialReadByte(){
    // Not Tested
    // Wait until RXNE = 1 (indicates DR ready to be read)
    while(!(USART1->SR & 0x00000020));
    uint8_t readData = (uint8_t)(USART1->DR & 0xFF); // Read Data
    return(readData);
}

void SerialBufferSend(volatile struct Buffer *serial_tx_buffer){
    // Send the contents of the serial_tx_buffer
    uint8_t tempCharStorage; // Location in memory to store the byte to send
    // While there is data in the buffer, read a byte from the buffer
    // and store it in tempCharStorage. Send this byte via the serial port.

    while(bufferRead(serial_tx_buffer, &tempCharStorage) == 0){
        SerialSendByte(tempCharStorage);
    }
}

void SerialSendString(uint8_t *array, uint8_t array_length){
    for(uint8_t i = 0; i<array_length; ++i){
        SerialSendByte(array[i]);
    }
}




