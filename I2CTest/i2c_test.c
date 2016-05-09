// STM32F103 I2C Setup & Test

#include "stm32f103xb.h"


void ClockSetup();
void I2CSetup();

void delay(int count)
{
    // volatile so that the compiler doesn't optimise it out
    volatile int i;

    for (i = 0; i < count; i++)
    {
    }
}

// Main - Called by the startup code.
int main(void) {
    ClockSetup();
    //MCO(); // Working - MCO = 8MHZ
    while(1){
    delay(1000);
    };
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
    RCC->APB2ENR |= 0x00000005; // GPIOA Clock Enable & Alt. Func Clk Enable
    GPIOA->CRH &= ~0x0000000F;
    GPIOA->CRH |= 0x0000000B; // PA8 alt config as MCO, 50MHz Max, Push-Pull O/P

    // MCO should be 36MHz - (MCO Pin Max 50MHz)
    RCC->CFGR |= 0x07000000; // Set Main Clk Output to PLL/2
}

void I2CSetup() {
    // ABP1 Bus Speed should be 36MHz



}

void I2CRead(uint8_t NumberBytesToRead, uint8_t slaveAddress)
{
    uint16_t flag_reset = 0;
    uint8_t Buffer[255]; // Define Buffer
    uint8_t *pBuffer = Buffer; // Pointer to Buffer start location

    // TODO: Add a check to see if NumberBytesToRead > Buffer to prevent overflow.

    if (NumberBytesToRead == 1){

        // Start
        // Send Slave Addr

        I2C->CR1 &= ~(0x0400); // Clear ACK Flag
        __disable_irq();

        // Clear Addr Flag
        while(!(I2C->SR1 & 0X0002)); // Read SR1 & Check for Addr Flag
        flag_reset = I2C->SR2; // Read SR2 to complete Addr Flag reset.

        I2C->CR1 |= 0x0200; // Set Stop Flag
        __enable_irq();

        while(!(I2C->SR1 & 0x0040)); // Wait for RxNE
        *pBuffer = I2C->DR; // Read Byte into Buffer
        pBuffer++; // Increment Buffer Pointer

        while (I2C->CR1 & 0x0200); // Wait until STOP Flag cleared by HW
        I2C->CR1 |= (0x0400); // Set ACK



    } else if (NumberBytesToRead == 2) {
        // Start
        // Send Slave Addr

        I2C->CR1 |= 0x0800; // Set POS Flag
        __disable_irq();

        // Clear Addr Flag
        while(!(I2C->SR1 & 0X0002)); // Read SR1 & Check for Addr Flag
        flag_reset = I2C->SR2; // Read SR2 to complete Addr Flag reset.


        I2C->CR &= ~(0x0400); // Clear ACK Flag
        __enable_irq();

        while(!(I2C->SR1 & 0x0004)); // Wait BTF Flag (Byte Transfer Finished)
        __disable_irq();

        I2C->CR1 |= 0x0200; // Set Stop Flag

        // Read 1st Byte
        *pBuffer = I2C->DR; // Read 1st Byte into Buffer
        pBuffer++; // Increment Buffer Pointer

        __enable_irq();

        *pBuffer = I2C->DR; // Read 2nd Byte into Buffer
        pBuffer++; // Increment Buffer Pointer

        while (I2C->CR1 & 0x0200); // Wait until STOP Flag cleared by HW

        // Clear POS Flag, Set ACK Flag (to be ready to receive)
        I2C->CR1 &= ~(0x0800); // Clear POS
        I2C->CR1 |= (0x0400); // Set ACK


    } else {
    // Read 3+ Bytes
    }
}


