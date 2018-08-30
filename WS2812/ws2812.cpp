// processloop.cpp
// Rewrite of I2CTest Code to create the appearance of concurrent operation of 
// the stepper motor, oled display and sensor readings without an RTOS. 
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>             // Enable fixed width integers
#include <stdio.h>              // Newlib-nano
#include "buffer_class.h"       // Circular Buffer Class
#include "clock.h"              // Setup system and peripheral clocks
#include "delay.h"              // Simple Delay Function
#include "stm32f103xb.h"        // HW Specific Header

////////////////////////////////////////////////////////////////////////////////
// Function Declarations
extern "C" void SysTick_Handler(void);
extern "C" void TIM2_IRQHandler(void);
void SysTick_Init(void);
void delay_ms(uint32_t ms);
void toggleLed();
void Timebase_Setup();
////////////////////////////////////////////////////////////////////////////////
// Buffers
// -------
// Create buffers - Size defined by buffer_class.h or variable for compiler.
//Buffer serial_rx;
//Buffer serial_tx;
//Buffer rx_buffer;
////////////////////////////////////////////////////////////////////////////////
// Global Variables
volatile uint32_t ticks = 0;        // Used for SysTick count down.
volatile uint32_t flash = 0;        // Used for PC13 LED Flash Toggle Interval
////////////////////////////////////////////////////////////////////////////////
// Main - Called by the startup code.
int main(void) {
    ClockSetup();       // Setup System & Peripheral Clocks
    SysTick_Init();     // Enable SysTick
    Timebase_Setup();  
    // PortA GPIO Setup
    // Enable I/O Clock for PortA
    //RCC->APB2ENR |= 0x00000004;
    // Configure PA0-PA3 
    // - General Purpose Output Push-Pull
    // - Max Speed 50MHz
        
    //GPIOA->CRL &= 0xFFFF0000; // Zero Settings for PA0-PA3, preserve PA4-PA7
    //GPIOA->CRL |= 0x00002222; // Apply Config to PA0-PA3 (2MHz)
    //GPIOA->CRL |= 0x00003333; // Apply Config to PA0-PA3 (50MHz)


    // PortC GPIO Setup
    // Enable I/O Clock for PortC
    RCC->APB2ENR |= 0x00000010;

    // Configure PC13
    // - LED Indicator
    // - General Purpose Output Push-Pull
    // - Max Speed 50MHz
    
    GPIOC->CRH &= 0xFF0FFFFF; // Zero Settings for PC13, preserve the rest
    GPIOC->CRH |= 0x00300000; // Apply Config to PC13 (50MHz)
    
    

    while(1){
        toggleLed();    // Toggle LED (PA13)  to indicate loop operational
    }
}

void TIM2_IRQHandler(void){
    // Timer 2 Interrupt Handler
    TIM2->SR &= 0xFFFE;         // Clear UIE Flag (Update Interrupt Enable)
    GPIOA->ODR ^= 0x00000001;   // Toggle PA0
}

void Timebase_Setup(){
    // Setup a Timebase using a Timer and Interrupts
    // Using Timer 2 (PA0)
    
    // Enable Clocks - Port A, Alternate Function
    RCC->APB2ENR |= 0x00000005;
    
    // Enable Timer 2
    RCC->APB1ENR |= 0x00000001;
    
    // Configure PA0 as GPIO Push-Pull Output, 50MHz
    GPIOA->CRL &= 0xFFFFFFF0; // Clear Bits for PA0. Preserve Rest
    GPIOA->CRL |= 0x00000003; // Setup PA0
        
    // Timer_Period (Hz) = Timer_Clock / ((Prescaler + 1)(AutoReloadReg + 1)) 
    // Timer_Clock = 72MHz
    // If Prescaler = 575, AutoReloadReg = 62499
    // Then Timer_Period = 2Hz (500ms)
        
    // Setup Timer2 Auto Reload Value
    TIM2->ARR = 0xF423; // 62499
    
    // Setup Timer2 Presaler Value
    TIM2->PSC = 0x023F; // 575
    
    // Enable Interrupt
    NVIC_EnableIRQ(TIM2_IRQn); // IRQ 28
    TIM2->DIER |= 0x0001; // Update Interrupt Enabled
    TIM2->CR1 |=  0x0081; // Auto Preload Enable, Enable Timer Counter
}
void toggleLed(){
    // Toggle the LED attached to PA13
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

void SysTick_Init(void){
    // SystemCoreClock/1000     =  1ms
    // SystemCoreClock/100000   = 10us
    // SystemCoreClock/1000000  =  1us
    
    // Triggering the interupt every 1us is probably excessive unless there is a
    // specific reason. An interrupt every 1ms should be sufficient.
    
    // SysTick_Config() is defined in core_cm3.h
    // Value of SystemCoreClock is defined in startup file.
    
    // Setup and start SysTick
    while(SysTick_Config(SystemCoreClock/1000) != 0){
        // One SysTick Should now equal 1ms
        // This means the interrupt SysTick_Handler() will run every 1ms. 
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

void delay_ms(uint32_t ms){
    
    if (ms != 0xFFFFFFFF){
        // Increment the delay count by one to compensate for the pre-decrement
        // in the SysTick handler.
        // The only instance where this will not happen is when the delay has
        // been set to its maximum value. A 1ms difference over the maximum 
        // delay is insignificant and is ignored. A 1ms difference will be more 
        // apparent over shorter delays hence the need for compensation.
        ++ms;
    }
    ticks = ms;
    while(ticks);
    
}
