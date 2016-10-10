// STM32F103 Serial Setup & Test with Buffer
////////////////////////////////////////////////////////////////////////////////
#include "clock.h"      // Setup system and peripheral clocks
#include "buffer.h"
#include "serial.h"
#include <stdint.h>                     // uint8_t
#include "stm32f103xb.h"
////////////////////////////////////////////////////////////////////////////////
// Function Declarations
// ---------------------
// http://stackoverflow.com/questions/12543076/usart-receive-interrupt-stm32
extern "C" void USART1_IRQHandler(void); // Solves Interrupt Problem
void ClockSetup();  // Setup the system clock sources
void SerialSetup(); // Setup USART1
void delay(int);    // Simple Delay
void SerialReceiveEcho(); // Serial Test Function - Echos Received Data
volatile uint8_t flag_overrun = 0; // Monitors SerialRx Overrun

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
    ClockSetup();
    SerialSetup();

    // Strings
    uint8_t test_message[] = "Waiting!\n\r"; //Size 10, escape chars 1 byte each
    uint8_t bufferStatus = 1; // Start as 1 indicating empty buffer.
    uint8_t tmp_byte = 0x00;

    LoadBuffer(&serial_tx_buffer, test_message, 10);
    SerialBufferSend(&serial_tx_buffer);

    __enable_irq();  // Is this needed??

    while(1){ // Required to prevent SIGTRAP - Infinite loop.

        //SerialReceiveEcho(); // Polls RXNE for data (no interrupts)

        // Obtain status of rx buffer, Status 0 = holds data, 1 = empty
        bufferStatus = bufferPeek(&serial_rx_buffer,&tmp_byte);
        if (bufferStatus == 0){                     // If data in buffer
            SerialBufferSend(&serial_rx_buffer);    // Transmit buffer content
        }
    }
}

void SerialReceiveEcho(){
    // Check if RXNE flag in USART1->SR, read data if set.
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
/*
void ClockSetup() {
    RCC->CR |= 0x00000001; // Turn on HSI Oscillator (8MHz)
    while(!(RCC->CR & 0x00000002)); // Wait for HSI Oscillator to stabalise.

    RCC->CR = 0x00000083;   // Reset RCC->CR to default values.
    RCC->CFGR = 0x00000000; // Reset RCC->CFGR to default values.
    RCC->CIR = 0x00000000;  // Reset RCC->CIR to defaults, disables interrupts.

    RCC->CR |= 0x00010000;          // Turn on HSE Oscillator (8MHz)
    while(!(RCC->CR & 0x00020000)); // Wait for HSE Oscillator to stablalise.

    // Setup System for 72MHz Clock
    // PLL Clock = 8MHz * 9 = 72MHz
    // USB Clock = PLL Clock / 1.5 = 48MHz.
    // ABP1 Clock = 72 / 2 = 36MHz
    // ABP2 Clock = 72MHz / 1 = 72MHz (PPRE2 default value,not shown in code).
    RCC->CFGR |= 0x001C0000;        // Set PLLMUL: x9, USB Prescaller: PLL/1.5
    RCC->CFGR |= 0x00010000;        // Set HSE Osc as PLL Source
    RCC->CR |= 0x00000400;          // Set PPRE1: APB Low Speed PreScaler /2

    RCC->CR |= 0x01000000;          // Turn on PLL
    while(!(RCC->CR & 0x02000000)); // Wait for PLL to stabalise

    // Adjust Flash Latency for 72MHz SYSCLK.
    FLASH->ACR = 0x00000030; // Reset Flash Access Control Register to defaults
    FLASH->ACR |= 0x00000002; // Set Flash Latency to 2 wait states


    RCC->CFGR |= 0x00000002;          // Enable PLL as SYSCLK
    while(!(RCC->CFGR & 0x00000008)); // Wait for SYSCLK switch over to PLL


    // Configuring MCO not strictly required in this case but useful
    // for checking SYSCLK

    //Configure MCO (PA8)
    //RCC->APB2ENR |= 0x00000005; // GPIOA Clock Enable & Alt. Func Clk Enable
    //GPIOA->CRH &= ~0x0000000F;
    //GPIOA->CRH |= 0x0000000B; // PA8 alt config as MCO, 50MHz Max, Push-Pull O/P

    // MCO should be 36MHz - (MCO Pin Max 50MHz)
    RCC->CFGR |= 0x07000000; // Set Main Clk Output to PLL/2
    return;
}
*/

void delay(int count){
    // volatile so that the compiler doesn't optimise it out
    volatile int i;

    for (i = 0; i < count; i++)
    {
    }
}

void USART1_IRQHandler(void){

    // USART IRQ Triggered
    //  Read USART_SR & determine RXNE or ORE
    //  If overrun, increment flag_overrun, store the data to buffer and return.
    //  If no overrun read the received data to the buffer and return

    //if (USART1->SR & 0x00000008){  // Read Status register
    //    // If Overrun detected increment flag (ORE Set)
    //    flag_overrun = flag_overrun + 1;
    //}

    // Store byte in buffer
    if (USART1->SR & 0x00000020){ // Read Status register
        // If Read Data Register not empty (RXNE Set)
        uint8_t data = (uint8_t)(USART1->DR & 0xFF); // Read Data resgister, Reads lowest 8bits of 32.
        bufferWrite(&serial_rx_buffer, data);
    }
}




