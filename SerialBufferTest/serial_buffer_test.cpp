// STM32F103 Serial Setup & Test with Buffer
////////////////////////////////////////////////////////////////////////////////
#include "buffer.h"
#include "serial.h"
#include <stdint.h> // uint8_t
#include "stm32f103xb.h"

// Function Declarations
void ClockSetup();
void SerialSetup();

void delay(int);

void SerialReceiveEcho();
void SerialBufferReceive();
volatile uint8_t flag_overrun = 0; // Monitors SerialRx Overrun
volatile uint8_t flag_test = 0; // Test Flag
volatile struct Buffer serial_rx_buffer {{},0,0};
////////////////////////////////////////////////////////////////////////////////
// Main - This function is called by the startup code.
int main() {
    ClockSetup();
    SerialSetup();

    // Create buffers - Size defined by buffer.h or variable for compiler.
    volatile struct Buffer serial_tx_buffer {{},0,0};

    // Strings
    uint8_t test_message[] = "Waiting!\n\r"; //Size 10, escape chars 1 byte each

    //LoadBuffer(&serial_tx_buffer, test_message, sizeof(test_message));
    LoadBuffer(&serial_tx_buffer, test_message, 10);
    SerialBufferSend(&serial_tx_buffer);

    while(1){ // Required to prevent SIGTRAP - Infinite loop.

        // The loop will poll the RXNE bit, if set there is data to read.
        // If there is data, read it and transmit it back as confirmation.
        SerialReceiveEcho();


        /*
        // Send Buffer Status
        SerialSendByte(0x30 + a);   // 0x30 offset to push into ASCII number range
        SerialSendByte(0x20);       // Send Space

        if (flag_overrun == true){
            SerialSendByte(0x31);   // Send Overrun True
            flag_overrun = 0;       // Reset Flag
        }else{
            SerialSendByte(0x30);
        }

        SerialSendByte(0x20);       // Send Space
        SerialSendByte(0x30 + flag_test); // Send Flag Test value

        SerialSendByte(0x0A);       // Send Line Feed
        SerialSendByte(0x0D);       // Send CR
        delay(8000000);             //1 Second Delay

        // Send the contents of the serial_tx_buffer (Should be "Waiting!")
        SerialBufferSend(&serial_tx_buffer);
        //delay(80000000);            //10 Second Delay
        delay(40000000);            // 5 Second Delay
        //SerialBufferSend(&serial_rx_buffer);
        */
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

void SerialBufferReceive(uint8_t rx_data){
    // Receives data to the serial_rx_buffer
    // TODO: Setup the interrupts to trigger when a byte is received.
    // Store the received byte in the serial_rx_buffer

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

    // DEBUG: Disable USART Interrupts for the moment.
    //NVIC_SetPriority(USART1_IRQn,1); // Set Interrupt Priority
    //NVIC_EnableIRQ(USART1_IRQn); // IRQ 37
}

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

    if (USART1->SR & 0x00000008){  // Read Status register
        // If Overrun detected increment flag (ORE Set)
        flag_overrun = flag_overrun + 1;
    }

    // Store byte in buffer
    while (USART1->SR & 0x00000020){ // Read Status register
        // If Read Data Register not empty (RXNE Set)
        uint8_t data = (uint8_t)(USART1->DR & 0xFF); // Read Data resgister, Reads lowest 8bits of 32.
        bufferWrite(&serial_rx_buffer, data);
    }

    // Increment flag to show interrupt has triggered
    flag_test = flag_test + 1;
    //NVIC_ClearPendingIRQ(USART1_IRQn); // Makes no difference
}




