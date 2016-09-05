// STM32F103 Serial Setup & Test with Buffer
////////////////////////////////////////////////////////////////////////////////
#include "buffer.h"
#include <stdint.h> // uint8_t
#include "stm32f103xb.h"

// Function Declarations
void ClockSetup();
void SerialSetup();
void SerialSendByte(uint8_t);
uint8_t SerialReadByte();
void delay(int);
void SerialBufferSend(volatile struct Buffer &serial_tx_buffer);
void SerialBufferReceive();
////////////////////////////////////////////////////////////////////////////////
// Main - This function is called by the startup code.
int main() {
    ClockSetup();
    SerialSetup();

    // Create buffers - Size defined by buffer.h or variable for compiler.
    volatile struct Buffer serial_tx_buffer {{},0,0};
    //volatile struct Buffer serial_rx_buffer {{}, 0, 0};
    // Strings
    //uint8_t test_message[] = "Does it work?\n"; //Size 15
    uint8_t test_message[] = "Works!\n"; //Size 7

    while(1){ // Required to prevent SIGTRAP - Infinite loop.
        // Load Serial_TX_Buffer
        // The buffer is loaded with the contents of test_message[]
        // The status of the buffer is loaded into 'a'
        // The 'H' character is printer to the serial port to confirm operation
        // The buffer status 'a' is the printed.
        // Delay for readability.

        // Expected Output
        // ---------------
        // With test_message size 7 the output on the serial port will be:
        // H0H0H2H2H2H2.....
        // The code loops twice without the buffer being filled (H0H0)
        // from that point the buffer is full (H2H2H2H2...)
        // ----
        // With test_message size 15 the output on the serial port will be:
        // H2H2H2H2H2H2.....
        // The buffer is filled on the first pass through the loop (H2H2H2H2...)
        // ----

        //LoadBuffer(&serial_tx_buffer, test_message, sizeof(test_message));
        uint8_t a = LoadBuffer(&serial_tx_buffer, test_message, 7);
        SerialSendByte(0x48); //H
        // Send Buffer Status
        SerialSendByte(0x30 + a); // 0x30 offset to push into ASCII number range
        delay(8000000); //1 Second Delay
        
        // Send the contents of the serial_tx_buffer
        SerialBufferSend(&serial_tx_buffer);
        delay(8000000); //1 Second Delay
    }
}

void SerialBufferSend(volatile struct Buffer &serial_tx_buffer){
    // Send the contents of the serial_tx_buffer
    uint8_t tempCharStorage; // Location in memory to store the byte to send
    // While there is data in the buffer, read a byte from the buffer
    // and store it in tempCharStorage. Send this byte via the serial port.

    while(bufferRead(&serial_tx_buffer, &tempCharStorage) == 0){
        SerialSendByte(tempCharStorage);
    }
}

void SerialBufferReceive(){
    // Receive data to the serial_rx_buffer
}

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
    RCC-> APB2ENR |= 0x00004000;

    // Set Baud Rate to 9600 (Based on 72MHz Sysclk)
    USART1->BRR = 0x1D4C;

    // Enable USART
    USART1->CR1 |= 0x00002000;
    // Enable USART Tx (Auto sends idle frame??)
    USART1->CR1 |= 0x00000008;
    // Enable USAER Rx
    USART1->CR1 |= 0x00000004;
}

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

void delay(int count)
{
    // volatile so that the compiler doesn't optimise it out
    volatile int i;

    for (i = 0; i < count; i++)
    {
    }
}
