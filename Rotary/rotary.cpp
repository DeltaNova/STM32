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
void toggleLed();
void PC13_LED_Setup(); // Setup PC13 for output LED
void EncoderSetup();
void EncoderButtonSetup();
uint32_t get_upcounting_delta(uint32_t start_count, uint32_t stop_count);
uint16_t get_diff(uint16_t count, uint16_t last_count);
void update_encoder_counts();
void updateValue(uint16_t dir, uint16_t delta);

// USART1
Buffer serial_tx; // USART1 TX Buffer (16 bytes)
Buffer serial_rx; // USART1 RX Buffer (16 bytes)

// Systick Counter
volatile uint32_t counter = 0;

// Rotary Encoder
volatile uint16_t encoder_count = 0;
volatile uint16_t last_encoder_count = 0; 

// Rotary Encoder Button
volatile uint16_t buttonPressStart = 0;
volatile uint16_t buttonPressStop = 0;
volatile uint8_t buttonPressed = 0;
void buttonAction(Serial& serial);

uint8_t buttonMessage[]= "Button Pressed\n\r"; //Size 16
uint8_t buttonMessage2[]= "Short Press\n\r"; //Size 13
uint8_t buttonMessage3[]= "Long Press\n\r"; //Size 12
uint8_t buttonMessage4[]= "Very Long Press\n\r"; //Size 17
uint8_t buttonMessage5[]= "Button Released\n\r"; //Size 17

// TODO: Convert to struct
// Test Value with upper and lower limts.
uint8_t value = 0;
uint8_t valueMin = 0;
uint8_t valueMax = 255;

// TODO: Used by buttonAction(), rewrite to remove
char char_buffer2[16]; // DEBUG

// Button Debounce 
void update_button(uint32_t *button_history);

// BUTTON_MASK - Assuming 1ms history update allows for 16ms of switch bounce.
#define BUTTON_MASK 0b11111111000000000000000011111111

// History of button state.
uint32_t button_history = 0; 
////////////////////////////////////////////////////////////////////////////////
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
    EncoderButtonSetup(); // Setup the Rotary Encoder Button
    
    // Strings & Initial Values
    uint8_t test_message[] = "Waiting!\n\r"; //Size 10, escape chars 1 byte each

    char char_buffer[16];
    // Send a message to the terminal to indicate program is running.
    serial.write_array(test_message,10);
    serial.write_buffer();
    
    while(1){
                        // Triggers Every Second
        toggleLed();    // Toggle LED (PC13) to indicate loop operational
        
        // Assess what the button is doing and trigger appropritate action.
        buttonAction(serial);
        
        update_encoder_counts();
        
        if ((encoder_count/4) != (last_encoder_count/4)) {  // If encoder_count has changed
            // encoder_count & last_encoder_count values are divided by 4 before use.
            // This is to reflect the 4 clock pulses per detent.
            // The result is the following code executes every detent.
            
            
            // Write new encoder_count to serial port.
            snprintf(char_buffer, 8, "%05u", encoder_count); 
                for(uint8_t i=0;i<5; i++){
                    serial.write(char_buffer[i]);
                }
            
            serial.write(0x20); // SPACE
            
            // This second count (encoder_count/4) changes based on the detents.
            // This new count smooths out the clock pulses between detents
            // while also incrementing in single digits.
            
            snprintf(char_buffer, 8, "%05u", encoder_count/4); // Count/4
                for(uint8_t i=0;i<5; i++){
                    serial.write(char_buffer[i]);
                }
                
            serial.write(0x20); // SPACE
            
            
            // This third value is the count delta, used for debugging
            uint16_t delta = get_diff(encoder_count,last_encoder_count);
            snprintf(char_buffer, 8, "%05u", delta);
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

uint32_t read_button(void){
    // Read the button state - Return 1 for pressed, 0 for released.
    uint32_t button_state;
    if (GPIOB->IDR & 0x00000040){   // PB6 High (Not Pressed)
        button_state = 0;
    }else{                          // PB6 Low (Pressed)
        button_state = 1;
    }
    return button_state;
}
void update_button(uint32_t *button_history){   // Called by Systick every 1ms
    // Bit shift button history to make room for new reading.
    *button_history = *button_history << 1;
    // Read current button state into history.
    *button_history |= read_button();
}
uint8_t is_button_up(uint32_t *button_history){
    // Returns true if button up
    return(*button_history == 0x00000000); 
}
uint8_t is_button_down(uint32_t *button_history){
    // Returns true if button down
    return(*button_history == 0xFFFFFFFF);
}
uint8_t is_button_pressed(uint32_t *button_history){
    // Returns true if pressed
    uint8_t pressed = 0;
    if ((*button_history & BUTTON_MASK) == 0b00000000000000000000000011111111){
        pressed = 1;
        // Set History to prevent retriggering from same press
        *button_history = 0xFFFFFFFF; 
    }
    return pressed;
}
uint8_t is_button_released(uint32_t *button_history){
    // Returns true if released
    uint8_t released = 0;
    if ((*button_history & BUTTON_MASK) == 0b11111111000000000000000000000000){
        released = 1;
        // Clear Histtory to prevent retriggering from same release
        *button_history = 0x00000000; 
    }
    return released;
}

// TODO: Rewrite to accept char buffer as a variable
void buttonAction(Serial& serial){ 
    // Button Action from Polling
    if (is_button_pressed(&button_history)){
        // Reset Press Duration Counters to current counter value
        buttonPressStart = counter;
        buttonPressStop = counter;
        // Send Pressed Message
        serial.write_array(buttonMessage,16);
        serial.write_buffer();
    }
    if (is_button_released(&button_history)){
        buttonPressStop = counter;
        uint32_t buttondelta = get_upcounting_delta(buttonPressStart, buttonPressStop);
        // Send Released Message
        serial.write_array(buttonMessage5,17);
        serial.write_buffer();
        
        if (buttondelta < 1000){
            // Short Press
            serial.write_array(buttonMessage2,13);
        }else if (buttondelta <5000){
            // Long Press
            serial.write_array(buttonMessage3,12);
        }else{
            // Very Long Press
            serial.write_array(buttonMessage4,17);
        }
        serial.write_buffer();
        
        // Debug Code to Print Length of button press
        snprintf(char_buffer2, 12, "%010lu", buttondelta);
        for(uint8_t i=0;i<10; i++){
            serial.write(char_buffer2[i]);
        }
        serial.write(0x0A); // LF
        serial.write(0x0D); // CR
    }
}
void updateValue(uint16_t dir, uint16_t delta){
    // TODO: Rewrite to take value as a variable and return new value
    // Apply the delta to current value.
    uint16_t i = 0;
    if (dir){   // If TRUE then count DOWN
        for (i=0; i < delta; i++){
            if (value > valueMin){
                value--;
            }
        }
    }else{      // If FALSE then count UP
        for (i=0; i < delta; i++){
            if (value < valueMax){
                value++;    
            }
        }
    }
}

uint32_t get_upcounting_delta(uint32_t start_count, uint32_t stop_count){
    // Returns the difference between two count values.
    
    // DEV NOTE: Count values need to be be from an upcounting only counter.
    //           Decrementing counts can result in large deltas where only a
    //           small delta exits in reality.
    
    if (stop_count < start_count){
        // Counter has rolled over (at least once, multi rollover not handled)
        return ((0xFFFFFFFF - start_count) + 1 + stop_count);
    }else{
        return (stop_count - start_count);
    }
}
void EncoderButtonSetup(){
    // Button will be using PB6 and is wired in with a pullup resistor
    RCC->APB2ENR |= 0x00000009;     // Enable Clocks (AFIO, Port B)
    // Clear PB6 Bits Before Setting as register does not zero on reset.
    GPIOB->CRL &= 0xF0FFFFFF;
    GPIOB->CRL |= 0x08000000;       // Setup PB6 as an input
    GPIOB->ODR |= 0x00000040;       // Set PB6 to use Pullup
}
void EncoderSetup(){
    // Setup For Rotary Encoder
    // Using Timer3 with partial remap for use of PB4 & PB5
    RCC->APB2ENR |= 0x00000009;     // Enable Clocks (AFIO, Port B)
    AFIO->MAPR |= 0x00000800;       // Partial Remap of Timer 3 Ports
    RCC->APB1ENR |= 0x00000002;     // Enable Clocks (TIM3)
    
    // Configure PB4 & PB5 (Floating Input, Input Mode)
    GPIOB->CRL &= 0xFF00FFFF;       // Clear PB4/PB5 Bits
    GPIOB->CRL |= 0x00440000;       // Set ports as inputs   
        
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
    
    TIM3->SMCR |= 0x0003;           // Set SMS Bits for Encoder Mode 3
    
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
    TIM3->ARR = 0xFFFF;             // 65535
    
    //Enable Update Generation
    TIM3->EGR |= 0x0001;            // Reinitialise Counter & Update Registers
    TIM3->CR1 |= 0x00001;           // Enable Timer 3
}
uint16_t get_diff(uint16_t count, uint16_t last_count){
    // This function assumes a count range of 0x0000 to 0xFFFF with roll over
    // and roll under.
    
    // Note: It will not work correctly if the count is scaled before use.
    
    // A window is used to assess the difference between the count and 
    // last_count values. Values inside or outside of the window determine how 
    // the diff is calculated. Multiple rollover not handled.
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
void update_encoder_counts(){
    // Read the hardware counter connected to the rotary encoder.
    last_encoder_count = encoder_count;     // Store previous encoder_count 
    encoder_count = (uint16_t)TIM3->CNT;    // Read latest HW counter value
}
void PC13_LED_Setup(){
    // Configure PC13 LED Indicator
    // - General Purpose Output Push-Pull
    // - Max Speed 50MHz
    RCC->APB2ENR |= 0x00000010;     // Enable I/O Clock for PortC
    GPIOC->CRH &= 0xFF0FFFFF;       // Zero Settings for PC13, preserve the rest
    GPIOC->CRH |= 0x00300000;       // Apply Config to PC13 (50MHz)
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
    counter++;
    update_button(&button_history); // Log button state to debounce history
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
