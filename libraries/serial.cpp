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

void SerialReceiveEcho(){
    // Checks if RXNE flag in USART1->SR, read data if set.
    // Echoes data back to sending terminal.

    while (USART1->SR & 0x00000020){ // Read SR, check if RXNE Set
        uint8_t data = USART1->DR;
        // Echo Data to Terminal
        SerialSendByte(data);       // Return Rx Byte
        SerialSendByte(0x0A);       // Send Line Feed
        SerialSendByte(0x0D);       // Send CR
        // Store Data In Buffer
        bufferWrite(&serial_rx_buffer,data);
        bufferWrite(&serial_rx_buffer,0x0A);
        bufferWrite(&serial_rx_buffer,0x0D);
        // Send back contents of the rx_buffer.
        SerialBufferSend(&serial_rx_buffer);
    }
}

void SerialSetup(){
    // USART1 using PA9 - Tx, PA10 - Rx
    // Baud 9600,N,8,1 without HW flow control
    // Assume 72MHz Sysclk

    // Enable GPIOA Clock & Alternate Function Clock
    RCC->APB2ENR |= 0x00000005;

    // NOTE: Could Combine the following into a single statement.
    // Set PA9 as Push-Pull Output, Alternate Function, Low Speed (2MHz)
    GPIOA->CRH &= ~0x000000f0; // Reset PA 9 bits before setting on next line
    GPIOA->CRH |= 0x000000a0;
    // Set PA10 as Floating Input
    GPIOA->CRH &= ~0x00000f00; // Reset PA10 bits before setting on next line
    GPIOA->CRH |= 0x00000400;

    // Enable ABP USART Clock
    RCC->APB2ENR |= 0x00004000;

    // Set Baud Rate to 9600 (Based on 72MHz Sysclk)
    USART1->BRR = 0x1D4C;

    // Enable USART
    USART1->CR1 = 0x00002000;
    // Enable USART Tx (Auto sends idle frame??)
    USART1->CR1 |= 0x00000008;
    // Enable USART Rx
    USART1->CR1 |= 0x00000004;

    // USART1 Interrupts
    // -----------------
    // Only the RXNEIE interrupt will be enabled, this will trigger on:
    //      RXNE - Received data ready to be read
    //      ORE  - Overrun error detected
    // All other USART1 interrupts will be disabled.
    USART1->CR1 |= 0x00000020;
    USART1->CR2 = 0x00000000; // Default Values to prevent IRQ
    USART1->CR3 = 0x00000000; // Default Values to prevent IRQ

    NVIC_SetPriorityGrouping(3); // Set Group 4
    NVIC_SetPriority(USART1_IRQn,1);
    NVIC_EnableIRQ(USART1_IRQn); // IRQ 37
}


