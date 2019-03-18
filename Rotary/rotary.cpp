// rotary.cpp
// Program to use a rotart encoder
//  
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>             // Enable fixed width integers
#include <stdio.h>              // Newlib-nano
#include "clock.h"              // Setup system and peripheral clocks
#include "buffer_class.h"       // Cicular Buffers
#include "serial.h"             // USART1 Setup & Functions
#include "stm32f103xb.h"        // HW Specific Header
#include "systick.h"            // SysTick Configuration
extern volatile uint32_t ticks; // SysTick Library
////////////////////////////////////////////////////////////////////////////////
// Global Variables
volatile uint32_t flash = 0;        // Used for PC13 LED Flash Toggle Interval
////////////////////////////////////////////////////////////////////////////////
// Function Declarations
extern "C" void USART1_IRQHandler(void);
extern "C" void SysTick_Handler(void);
//extern "C" void TIM3_IRQHandler(void);

void toggleLed();
void PC13_LED_Setup(); // Setup PC13 for output LED
void EncoderSetup();

Buffer serial_tx; // USART1 TX Buffer (16 bytes)
Buffer serial_rx; // USART1 RX Buffer (16 bytes)

//volatile uint8_t flag = 0; // Flag used by TIM3 IRQ

volatile uint16_t count = 0;
// Create an initial difference in counter values to try and force a start
// value to be output on the serial port.
volatile uint16_t last_count = 1; 


// Main - Called by the startup code.
int main(void) {
    ClockSetup();       // Setup System & Peripheral Clocks
    SysTick_Init();     // Enable SysTick
    // Create instance of Serial class (USART1) 
    Serial serial(serial_rx, serial_tx);     
    serial.setup();     // Enable Serial Support - Currently USART1 Specific

    // NOTE: Tie USART RX Pin low as I suspect interrupt causing problems.

    
    PC13_LED_Setup();   // Setup PC13 for output LED
    EncoderSetup();     // Setup Rotary Encoder
    
    // Strings & Initial Values
    uint8_t test_message[] = "Waiting!\n\r"; //Size 10, escape chars 1 byte each
    char char_buffer[10];
    // Send a message to the terminal to indicate program is running.
    serial.write_array(test_message,10);
    serial.write_buffer();


    
    while(1){
        // Triggers Every Second
        toggleLed();    // Toggle LED (PC13) to indicate loop operational
        
        count = (uint16_t)TIM3->CNT; // Read current count.
        
        if (count != last_count) {  // If count has changed
            // Write new count to serial port.
            snprintf(char_buffer, 8, "%05u", count); 
                for(uint8_t i=0;i<5; i++){
                    serial.write(char_buffer[i]);
                }
            /*
            // This third count (IRQ flag) changes based on TIM3 IRQ.
            // Its purpose is to confirm that the IRQ triggers and is handled.
            
            snprintf(char_buffer, 8, "%05u", flag);
                for(uint8_t i=0;i<5; i++){
                    serial.write(char_buffer[i]);
                }
                    
            */    
            serial.write(0x0A); // LF
            serial.write(0x0D); // CR
            
        
        last_count = count; // Update Last Count
        }
    
    }
}

void EncoderSetup(){
    // Setup For Rotary Encoder
    // Using Timer3 with partial remap for use of PB4 & PB5
    
    // Enable Clocks (AFIO, Port B)
    RCC->APB2ENR |= 0x00000009;
    
    // Partial Remap of Timer 3 Ports
    AFIO->MAPR |= 0x00000800;
    // Enable Clocks (TIM3)
    RCC->APB1ENR |= 0x00000002;
    
    /*
    // Setup TIM3 Interrupts
    //TIM3->DIER |= 0x0001; // UIE (Update Interupt Enabled)
    TIM3->DIER |= 0x0040; // TIE (Trigger Interupt Enabled)
    */
    
    // Configure PB4 & PB5 (Floating Input, Input Mode)
    GPIOB->CRL &= 0xFF00FFFF;   // Clear PB4/PB5 Bits
    GPIOB->CRL |= 0x00440000;   // Set ports as inputs   
        
    // Setup Timer 3
    // CKD = 00 (Div/1)
    // ARPE = 0 (Not Bufffered)
    // CMS = 00 (Edge Alighed)
    // DIR = 0 (Up Counter)
    // OPM = 0 (Counter Not Stopped at Update Event)
    // URS = 0 (Update Request Sources)
    // UEV = 0 (Update Events Enabled)
    // CEM = 0 (Counter Disabled) <- Gets Enabled Later
    TIM3->CR1 = 0x0000;
    
    /*
    //TIM3->SMCR |= 0x0050; // Trigger Selection for interrupt, TI1FP1
    TIM3->SMCR |= 0x0060; // Trigger Selection for interrupt, TI1F_ED
    */
    
    // Set SMS Bits for Encoder Mode 3
    TIM3->SMCR |= 0x0003;
    
    // Ensure CC1E = 0 , CC2E = 0 to allow setting of capture/compare bits.
    TIM3->CCER &= 0xFFEE;
        
    // Set Capture/Compare Mode
    // CC1S = 01, CC2S = 01 (IC1 is mapped on TI1, IC2 is mapped on TI2)
    // IC1F = 0000, IC2F = 0000 (No filtering)
    // IC1PSC = 10, IC2PSC = 10 (Capture Every 4 Events)
    TIM3->CCMR1 |= 0x0909;
    
    // Set Polarity and Enable Capture/Compare
    // CC1P = 0, CC2P = 1
    // CC1E = 1, CC2E = 1
    TIM3->CCER |= 0x0031;
    
    // Set Max Count Value (Count up between 0 and ARR, count down ARR to 0)
    TIM3->ARR = 0xFFFF; // 65535
    
    /*
    //NVIC_SetPriority(TIM3_IRQn, 0, 1);
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
    */
    
    //Enable Update Generation
    TIM3->EGR |= 0x0001; // Reinitialise Counter & Update Registers
    
    // Enable Timer 3
    TIM3->CR1 |= 0x00001;    
}
/*
void TIM3_IRQHandler(void){
    //if ((TIM3->SR && 0x00001)){ // If UIF SET
    //    ++flag; // Increment Flag
    //    TIM3->SR &= ~0x0001; // Clear UIF
    //}
    
    if ((TIM3->SR && 0x0040)){ // If TIF SET
        if ((TIM3->CR1 && 0x0010)) { //If DIR Set (Down Count)
            --flag; // Decrement Flag
        }else{
            ++flag; // Increment Flag
        }
        TIM3->SR &= ~0x0040; // Clear TIF
    }
}
*/

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
