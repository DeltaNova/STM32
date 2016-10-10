// STM32F103 Serial Setup & Test with Buffer
////////////////////////////////////////////////////////////////////////////////
#include "clock.h"       // Setup system and peripheral clocks
#include "buffer.h"      // Circular Buffers
#include "serial.h"      // USART1 Setup & Functions
#include <stdint.h>      // uint8_t
#include "stm32f103xb.h" // HW Specific Header
////////////////////////////////////////////////////////////////////////////////
// Function Declarations
// ---------------------
// http://stackoverflow.com/questions/12543076/usart-receive-interrupt-stm32
// 'extern "C"' Solves Interrupt Problem of using C++ Compiler for C Code
extern "C" void USART1_IRQHandler(void);

////////////////////////////////////////////////////////////////////////////////
// Buffers
// -------
// Create buffers - Size defined by buffer.h or variable for compiler.
volatile struct Buffer serial_tx_buffer {{},0,0};
volatile struct Buffer serial_rx_buffer {{},0,0};

////////////////////////////////////////////////////////////////////////////////
// Main - This function is called by the startup code.
// ---------------------------------------------------
int main() {
    ClockSetup();       // Setup System & Peripheral Clocks
    SerialSetup();      // Enable Serial Support - Currently USART1 Specific

    // Strings & Initial Values
    uint8_t test_message[] = "Waiting!\n\r"; //Size 10, escape chars 1 byte each
    uint8_t bufferStatus = 1; // Start as 1 indicating empty buffer.
    uint8_t tmp_byte = 0x00;

    // Send a message to the terminal to indicate program is running.
    LoadBuffer(&serial_tx_buffer, test_message, 10);
    SerialBufferSend(&serial_tx_buffer);

    while(1){ // Program Loop - Checks if data has been received.

        // Obtain status of rx buffer, Status 0 = holds data, 1 = empty
        bufferStatus = bufferPeek(&serial_rx_buffer,&tmp_byte);

        if (bufferStatus == 0){                  // If data in buffer...
            SerialBufferSend(&serial_rx_buffer); // ...transmit buffer contents.
        }
    }
}

void USART1_IRQHandler(void){
    // USART IRQ Triggered
    if (USART1->SR & 0x00000020){ // Read Status reg, check if RXNE Set
        // Read Rx Data register, Reads lowest 8bits of 32.
        uint8_t data = (uint8_t)(USART1->DR & 0xFF);
        bufferWrite(&serial_rx_buffer, data);
    }
}
