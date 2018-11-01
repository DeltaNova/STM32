// ws2812.cpp
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
//extern "C" void TIM2_IRQHandler(void);
extern "C" void DMA1_Channel5_IRQHandler(void);
void SysTick_Init(void);
void delay_ms(uint32_t ms);
void toggleLed();
void PWM_Setup();      // Timer 2 PWM - Ch1 & Ch2
void DMA_Setup();
void Timebase_Setup(); // Timebase from Timer using interrupts
void PC13_LED_Setup(); // Setup PC13 for output LED
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

// Array of values to be transferred by DMA to TIM2->CCR1
uint8_t pwm_array[] = {
    0x09, 0x09, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // G = 63
    0x09, 0x09, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // R = 63
    0x09, 0x09, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // B = 63

    0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, // G = 0
    0x09, 0x09, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // R = 63
    0x09, 0x09, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // B = 63
    
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 48 no pwm cycles
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Reset 60uS 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       
};
////////////////////////////////////////////////////////////////////////////////
// Main - Called by the startup code.
int main(void) {
    ClockSetup();       // Setup System & Peripheral Clocks
    SysTick_Init();     // Enable SysTick
    PC13_LED_Setup();   // Setup PC13 for output LED
    DMA_Setup();
    PWM_Setup();

    // The counter will count to the reload value where upon it will toggle the
    // output state of each channel back to its reset value. 
    // During the count, before the reload value is reached, the count is 
    // compared with each channels compare value. If the compare value is
    // matched the output state of the matched channel will be toggled.
    
    // Timer 2 Channel 1 Compare Value
    //TIM2->CCR1 = 0x000F;        // 15 (Logic 1)
    TIM2->CCR1 = 0x0000; 
    // Timer 2 Channel 2 Compare Value
    TIM2->CCR2 = 0x0009;        // 9 (Logic 0)

    while(1){
        toggleLed();    // Toggle LED (PC13)  to indicate loop operational
    }
}

void DMA_Setup(){
    // DMA Setup - DMA1 Channel 5 for use with TIM2 Channel 1
    RCC->AHBENR |= 0x00000001; // Enable DMA 1 Clock
    
    // DMA Channel 5 Configuration
    // Load base address of peripheral register to which data will be written.
    // Load the address of the compare register converting from 16 to 32 bits
    DMA1_Channel5->CPAR = (uint32_t)(&(TIM2->CCR1));
    
    // Load the memory address data will be read from
    DMA1_Channel5->CMAR = (uint32_t)pwm_array;
    
    // Load the number of data transfers.
    // Circular mode will be used to loop over the two values in the array
    // so only to values need to be transferred before everyting resets.
    //DMA1_Channel5->CNDTR = 2;
    
    DMA1_Channel5->CNDTR = 96; // 24bits (GRB) + 24bits(GRB) + 48bits (Reset)
    
    // Configure Channel Priority
    // Leaving at default 'Low' priority
    //DMA-CCR5 |= 0x00000000; 
    
    // Configure additional channel features
    //  Memory Size 8 bits
    //  Peripheral Size 16 bits
    //  Memory Increment Mode Enabled
    //  Circular Mode Enabled
    //  Direction: Read From Memory
    DMA1_Channel5->CCR |= 0x000001B2;
    
    NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn);    // Clear Pending Status
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);          // Enable Interrupt
        
    // Activate Channel
    DMA1_Channel5->CCR |= 0x00000001;
    
}
void DMA1_Channel5_IRQHandler(void){

    // If Channel 5 Transfer Complete, Disable Timer 2 and DMA 1 Channel 5
    if (DMA1->ISR & 0x00020000){ 
        // Disable Timer 2
        TIM2->CR1 &= ~0x0001; // Clear Enable Bit
        // Disable DMA1 Channel 5
        DMA1_Channel5->CCR &= ~0x00000001; // Clear Enable Bit 
        // Failing to 
        DMA1->IFCR = 0x00020000; // Reset Transfer Complete Flag
    }
    
}

void PWM_Setup(){
    // Setup PWM using Timer2 (PA0, PA1)
    
    // Enable Clocks - Port A, Alternate Function
    RCC->APB2ENR |= 0x00000005;
    
    // Enable Timer 2
    RCC->APB1ENR |= 0x00000001;
    
    // Configure PA0, PA1 as AF Push-Pull Output Compare, 50MHz
    // Note: Reset Value For GPIOA-> is 0x44444444. Be careful when setting up
    // as incorrect function maybe selected.
    GPIOA->CRL = 0x000000BB;
    
    // Values Based on some example code
    // System Clock 72MHz
    // Timer Period (1/800kHz) = 0.00000125 seconds
    
    // SysClk/Timer Period = (Prescaler + 1)(Auto Reload Value +1)
    // 72x10^6/800x10^3 = 90
    
    // The values for the Prescaler and ARR are subject to interpretation on the 
    // ammount of steps (level of control) required over the output. The values
    // that follow are based on the example code of others and appear to have
    // been selected to make the maths easier.
    
    // Let Prescaler = 2
    
    // 90 = (Prescaler + 1)(Auto Reload Value +1)
    // 90 = 3(Auto Reload Value +1)
    // 90/3 = 30 = Auto Reload Value + 1
    // Auto Reload Value = 29
        
    // Set Timer 2 Reload Value
    TIM2->ARR = 0x001D; //29
    
    // Set Timer2 Prescaler Value
    TIM2->PSC = 0x0002; //2
    
    // Channel 1 & 2 - PWM Mode 2: 
    // Ch in upcounting is inactive as long as TIM2_CNT < TIM2_CCR2 else active.
    // Ch in downcounting is inactive as long as TIM2CNT>TIM2_CCR1 else active.
    TIM2->CCMR1 = 0x7878;
    // Alternatively
    // Channel 1 & 2 - PWM Mode 1: 
    // Ch in upcounting is active as long as 
    // TIM2_CNT<TIM2_CCR1
    // TIM2->CCMR1 = 0x6800;
    
    
    //Enable Update Generation
    TIM2->EGR |= 0x0001; // Reinitialise Counter & Update Registers
    
    // Channel 1 Enable
    TIM2->CCER |= 0x0003; // 0x0003 for active low, 0x0001 for active high
    
    // Channel 2 Enabled
    TIM2->CCER |= 0x0030; // 0x0030 for active low, 0x0010 for active high
    
    TIM2->CR2 |= 0x0004; // DMA Requests Sent when update event occurs.
    
    TIM2->DIER |= 0x0300; // Update DMA Request Enable, Ch 1 DMA Request Enable
    
    // Auto Preload Enable & Enable Timer Counter
    TIM2->CR1 |= 0x0081;
    
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
