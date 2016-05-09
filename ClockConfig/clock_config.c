// Basic STM32 demo that runs from the HSI (High Speed Internal) oscillator on
// power up and switches over to use the HSE (High Speed External) oscillator.

#include "stm32f103xb.h"

#include <stdio.h> // Provides printf (Semihosting)

#ifdef DEBUG
extern void initialise_monitor_handles(void);
#endif

void ClockSetup();
void MCO();
void LSE_Setup();

void semihost(void) {
    // Initialise Semihost requirement based on DEBUG flag setting
    #ifdef DEBUG
    initialise_monitor_handles();
    #endif
}

void semihostmsg(uint8_t msg) {
    // Output Selected Semihost message
    #ifdef DEBUG
    switch(msg){
    case 0:
        printf("Hello World\n");
        break;
    case 1:
        printf("Debug 1\n");
        break;
    default:
        printf("Invalid Message\n");
    }
    #endif
}

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
    semihost(); // Add semihost functionality
    ClockSetup();
    //MCO(); // Working - MCO = 8MHZ
    //LSE_Setup(); // Work In Progress
    semihostmsg(0); // Send Semihost Message
    while(1){
    delay(1000);
    semihostmsg(0);
    };
}

void LSE_Setup(){
    // TODO: Suspect this needs a few more register options to work.
    //      - Configure pins??
    // Try DBP bit in PWR_CR, ref 7.3.9 RCC_BDCR
    //  - Appears backup domain write protected after reset.

    RCC->APB1ENR = 0x00000000; // Reset RCC-APB1ENR to defaults
    RCC->APB1ENR |= 0x08000000;// Enable backup interface clock
    RCC->BDCR = 0x00000000; // Reset RCC->BDCR to defaults.

    RCC->BDCR |= 0x00000001;            // Turn on LSE Oscillator
    while(!(RCC->BDCR & 0x00000002));   // Wait until LSE ready & stable

    RCC->BDCR |= 0x00000100;    // Select LSE as RTC Source
    RCC->BDCR |= 0x00008000;    // Enable RTC
}

void ClockSetup() {
    RCC->CR |= 0x00000001; // Turn on HSI Oscillator
    while(!(RCC->CR & 0x00000002)); // Wait for HSI Oscillator to stabalise.

    RCC->CR = 0x00000083;   // Reset RCC->CR to default values.
    RCC->CFGR = 0x00000000; // Reset RCC->CFGR to default values.
    RCC->CIR = 0x00000000;  // Reset RCC->CIR to defaults, disables interrupts.

    RCC->CR |= 0x00010000;          // Turn on HSE Oscillator
    while(!(RCC->CR & 0x00020000)); // Wait for HSE Oscillator to stablalise.

    RCC->CFGR |= 0x001C0000;        // Set PLLMUL: x9, USB Prescaller: PLL/1.5
    RCC->CFGR |= 0x00010000;        // Set HSE Osc as PLL Source
    RCC->CR |= 0x00000400;          // Set PPRE1: APB Low Speed PreScaler /2
    RCC->CR |= 0x01000000;          // Turn on PLL
    while(!(RCC->CR & 0x02000000));  // Wait for PLL to stabalise

    FLASH->ACR = 0x00000030; // Reset Flash Access Control Register to defaults
    FLASH->ACR |= 0x00000002; // Set Flash Latency to 2 wait states
                              //based on sysclk value (48MHZ < SYSCLK < 72MHZ)

    RCC->CFGR |= 0x00000002; // Enable PLL as sysclk
    while(!(RCC->CFGR & 0x00000008)); // Wait for switch over to PLL Sysclk
    semihostmsg(1);

    //Configure MCO (PA8)
    //RCC->AHBENR |= 0x00000003; // Enable DMA1, DMA2  <-- Required?
    RCC->APB2ENR |= 0x00000005; // GPIOA Clock Enable & Alt. Func Clk Enable
    GPIOA->CRH &= ~0x0000000F;
    GPIOA->CRH |= 0x0000000B; // PA8 alt config as MCO, 50MHz Max, Push-Pull O/P

    // MCO should be 36MHz - (MCO Pin Max 50MHz)
    RCC->CFGR |= 0x07000000; // Set Main Clk Output to PLL/2
}

void MCO(){

    RCC->CR =   0x00000083; // Reset RCC->CR to default values.
    RCC->CFGR = 0x00000000; // Reset RCC->CFGR to default values.
    RCC->CIR =  0x00000000; // Reset RCC->CIR to defaults, disables interrupts.
    RCC->APB2ENR  = 0x00000014; // Reset RCC->APB2ENR default values.
    RCC->APB2ENR |= 0x00000005; // GPIOA Clock Enable & Alt. Func Clk Enable
    GPIOA->CRH &= ~0x0000000F;  // Zero PA8 config register bits
    GPIOA->CRH |= 0x0000000B; // PA8 alt config as MCO, 50MHz Max, Push-Pull O/P
    RCC->CFGR |= 0x05000000;  // MCO = HSI OSC (8MHz)
}
