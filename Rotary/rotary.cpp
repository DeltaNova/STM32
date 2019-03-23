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
volatile uint32_t count_update = 0;  // Used to time execution of update_counts() 
////////////////////////////////////////////////////////////////////////////////
// Function Declarations
extern "C" void USART1_IRQHandler(void);
extern "C" void SysTick_Handler(void);
void toggleLed();
void PC13_LED_Setup(); // Setup PC13 for output LED
void EncoderSetup();
uint16_t get_count_delta(uint16_t count, uint16_t last_count);
uint16_t get_diff(uint16_t count, uint16_t last_count);
void update_counts();
void updateValue(uint16_t dir, uint16_t delta);

Buffer serial_tx; // USART1 TX Buffer (16 bytes)
Buffer serial_rx; // USART1 RX Buffer (16 bytes)


volatile uint16_t count = 0;
volatile uint16_t last_count = 0; 

// Test Value with upper and lower limts.
uint8_t value = 0;
uint8_t valueMin = 0;
uint8_t valueMax = 255;

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
        update_counts();
        
        // Dev Note: The fact that the counts are only updated periodically allows the following print block to execute multiple times. 
        //           This is due to the "count != last_count" statement remaing true until the next time the counts update. 
        
        if (count != last_count) {  // If count has changed
            // Write new count to serial port.
            snprintf(char_buffer, 8, "%05u", count); 
                for(uint8_t i=0;i<5; i++){
                    serial.write(char_buffer[i]);
                }
            
            serial.write(0x20); // SPACE
            
            // This second count (count/4) changes based on the detents.
            // This new count smooths out the clock pulses between detents
            // while also incrementing in single digits.
            
            snprintf(char_buffer, 8, "%05u", count/4); // Count/4
                for(uint8_t i=0;i<5; i++){
                    serial.write(char_buffer[i]);
                }
                
            serial.write(0x20); // SPACE
            
            
            // This third value is the count delta, used for debugging
            uint16_t delta = get_diff(count,last_count);
            snprintf(char_buffer, 8, "%05u", delta/4);
                for(uint8_t i=0;i<5; i++){
                    serial.write(char_buffer[i]);
                }
            
            serial.write(0x20); // SPACE
            
            
            // This fourth value is the Timer Direction Bit, used for debugging
            // Prints: (Due to BIN to DEC conversion)
            // 00000 for UP
            // 00016 for DOWN 
            uint16_t dir = (TIM3->CR1 & 0x0010);
            snprintf(char_buffer, 8, "%05u", dir);
                for(uint8_t i=0;i<5; i++){
                    serial.write(char_buffer[i]);
                }
            
            serial.write(0x20); // SPACE
            
            updateValue(dir, delta);    
            // This fifth value is the test value to be adjusted.
            // ValueMin = 0, ValueMax=255
            
            snprintf(char_buffer, 8, "%05u", value);
                for(uint8_t i=0;i<5; i++){
                    serial.write(char_buffer[i]);
                }
                              
            serial.write(0x0A); // LF
            serial.write(0x0D); // CR
        }
    
    }
}

void updateValue(uint16_t dir, uint16_t delta){
    // Apply the delta to current value.
    uint16_t i = 0;
    if (dir){   // If TRUE then count DOWN
        for (i; i < delta; i++){
            if (value > valueMin){
                value--;
            }
        }
    }else{      // If FALSE then count UP
        for (i; i < delta; i++){
            if (value < valueMax){
                value++;    
            }
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
    
    //Enable Update Generation
    TIM3->EGR |= 0x0001; // Reinitialise Counter & Update Registers
    
    // Enable Timer 3
    TIM3->CR1 |= 0x00001;    
}

uint16_t get_diff(uint16_t count, uint16_t last_count){
    uint16_t diff;
    if (count > last_count){
        if (count < 0x8000){
            return (count - last_count);            // Increment
        }else{ // (count >= 0x8000)
            if (last_count > (count-0x4000)){
                return (count - last_count);        // Increment
            }else{ // (last_count <= (count-0x4000))
                // Count Roll Under Condition
                diff = (0xFFFF - count);
                return(last_count + diff);          // Decrement
            }
        }
    }else{ // (count <= last count)
        if (last_count < 0x8000){
            return(last_count - count);             // Decrement
        }else{ // (last_count >= 0x8000)
            if (count > (last_count - 0x4000)){
                return(last_count - count);         // Decrement
            }else{ //(count <= (last_count - 0x4000))
                // Count Roll Over Condition
                diff = 0xFFFF - last_count;
                return (count + diff);              // Increment
            }
        }
    }
}

uint16_t get_count_delta(uint16_t count, uint16_t last_count){
    // Function to return the difference between two count values
    
    if (count < last_count){
        // Counter has rolled over (at least once, but multiples not handled).
        // +1 to handle zero position.
        return(0xFFFF - last_count) + 1 + count;   
    }else{
        return(count - last_count);
    }
}

void update_counts(){
    // Run periodically to update the count and last_count values.
    if (count_update == 0){
        last_count = count;          // Store previous count as last_count
        count = (uint16_t)TIM3->CNT; // Read HW counter value
        count_update = 0;  // DEBUG Changed to zero to make count value update faster. 
        // It might be better to remove this update function all together. 
        // However being able to slow things down might help debugging.
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
    
    if (count_update !=0){ // Decrement count_update counter
        --count_update;
    }
}
