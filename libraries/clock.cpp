// clock.cpp -- Clock Setup STM32F103

#include "stm32f103xb.h" // Need as direct reference to HW

void ClockSetup() {
    RCC->CR |= 0x00000001; // Turn on HSI Oscillator (8MHz)
    while(!(RCC->CR & 0x00000002)); // Wait for HSI Oscillator to stabalise.

    RCC->CR = 0x00000083;   // Reset RCC->CR to default values.
    RCC->CFGR = 0x00000000; // Reset RCC->CFGR to default values.
    RCC->CIR = 0x00000000;  // Reset RCC->CIR to defaults, disables interrupts.

    RCC->CR |= 0x00010000;          // Turn on HSE Oscillator (8MHz)
    while(!(RCC->CR & 0x00020000)); // Wait for HSE Oscillator to stablalise.

    // Setup System for 72MHz Clock
    // NOTE: SystemCoreClock = 72000000U variable stored in system_stm32f1xx.c
    // TODO: Add function to update SystemCoreClock variable.
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
    //RCC->CFGR |= 0x07000000; // Set Main Clk Output to PLL/2
    return;
}
