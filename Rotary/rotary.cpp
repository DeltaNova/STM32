// rotary.cpp
// Program to use a rotart encoder
//  
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>             // Enable fixed width integers
#include <stdio.h>              // Newlib-nano
#include "clock.h"              // Setup system and peripheral clocks
#include "stm32f103xb.h"        // HW Specific Header
#include "systick.h"            // SysTick Configuration
extern volatile uint32_t ticks; // SysTick Library
////////////////////////////////////////////////////////////////////////////////
// Global Variables
volatile uint32_t flash = 0;        // Used for PC13 LED Flash Toggle Interval
////////////////////////////////////////////////////////////////////////////////
// Function Declarations
extern "C" void SysTick_Handler(void);
void toggleLed();
void PC13_LED_Setup(); // Setup PC13 for output LED

// Main - Called by the startup code.
int main(void) {
    ClockSetup();       // Setup System & Peripheral Clocks
    SysTick_Init();     // Enable SysTick
    PC13_LED_Setup();   // Setup PC13 for output LED

    while(1){
        // Triggers Every Second
        toggleLed();    // Toggle LED (PC13) to indicate loop operational
    }
}

void PC13_LED_Setup(){
    // PortC GPIO Setup
    // Enable I/O Clock for PortC
    RCC->APB2ENR |= 0x00000010;

    // Configure PC13
    // - LED Indicator
    // - General Purpose Output Push-Pull
    // - Max Speed 50MHz
    
    GPIOC->CRH &= 0xFF0FFFFF; // Zero Settings for PC13, preserve the rest
    GPIOC->CRH |= 0x00300000; // Apply Config to PC13 (50MHz)
}
void toggleLed(){
    // Toggle the LED attached to PC13
    if (flash == 0){
        // Get LED Status
        if (GPIOC->ODR & 0x00002000){ // If Set
            GPIOC->BSRR |= 0x20000000; // Reset
        }else{                        // If Not Set
            GPIOC->BSRR |= 0x00002000; // Set
        }
        flash = 1001;
    }
}
void SysTick_Handler(void){
    if (ticks != 0){
        // Pre-decrement ticks. This avoids making a copy of the variable to 
        // decrement. This should be faster which is ideal for an interrupt
        // service routine.
        // Note: Since the value will pre-decrement the value of ticks will need
        // to be incremented by 1 to be correct.
        --ticks;
    }
    if (flash !=0){ // Decrement the LED Toggle Counter
        --flash;
    }
    
}
