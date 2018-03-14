// STM32F103 I2C Setup & Test
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>             // Enable fixed width integers
#include <stdio.h>              // Newlib-nano
#include "buffer_class.h"       // Circular Buffer Class
#include "clock.h"              // Setup system and peripheral clocks
#include "serial.h"             // USART1 Setup & Functions
#include "delay.h"              // Simple Delay Function
#include "i2c.h"                // I2C Setup and control functions
#include "rodent_buffer.h"      // Rodent Display Buffer
#include "test_pattern_buffer.h"// Test Pattern Display Buffer
#include "BH1750FVI.h"          // I2C Lux Sensor
#include "stm32f103xb.h"        // HW Specific Header

////////////////////////////////////////////////////////////////////////////////
// Definitions
#define OLED_ADDR   0x7a      // Address of I2C OLED Display
#define LUX_ADDR    0xB9      // Address of I2C Lux Sensor
////////////////////////////////////////////////////////////////////////////////
// Function Declarations
extern "C" void USART1_IRQHandler(void);
extern "C" void SysTick_Handler(void);
void SysTick_Init(void);
void delay_ms(uint32_t ms);
void OLEDSetup(I2C& i2c);
void draw_buffer2(I2C& i2c);
void draw_buffer3(I2C& i2c);
void clear_buffer(I2C& i2c);

////////////////////////////////////////////////////////////////////////////////
// Buffers
// -------
// Create buffers - Size defined by buffer_class.h or variable for compiler.
Buffer serial_rx;
Buffer serial_tx;
Buffer rx_buffer;
////////////////////////////////////////////////////////////////////////////////
// Global Variables
volatile uint32_t ticks = 0;        // Used for SysTick count down.

////////////////////////////////////////////////////////////////////////////////
// Main - Called by the startup code.
int main(void) {
    ClockSetup();       // Setup System & Peripheral Clocks
    SysTick_Init();     // Enable SysTick
   
    // Create instance of Serial class (USART1) 
    Serial serial(serial_rx, serial_tx);     
    serial.setup();     // Enable Serial Support - Currently USART1 Specific
    
    I2C i2c(rx_buffer);             // Create instance of I2C class (I2C1)
    i2c.I2C1Setup();                // Setup I2C1
    OLEDSetup(i2c);                 // Create instance of OLED Display
    BH1750FVI lux(i2c, LUX_ADDR);   // Create an instance of BH1750FVI Sensor
    lux.setup();                    // Setup BH1750FVI

    // USART1 Message to confirm program running - Using for Debugging
    uint8_t test_message[] = "Waiting!\n\r"; //Size 10, escape chars 1 byte each
    // Send a message to the terminal to indicate program is running.
    serial.write_array(test_message, 10);
    serial.write_buffer();

    while(1){
    // The total delay in the loop needs to be adjusted so that we dont end up
    // reading the lux sensor too often.
    longdelay(0xFFFF);  // Allow time for reading to be taken, auto power down.
    longdelay(0xFFFF);  // Allow time for reading to be taken, auto power down.

    lux.read();         // Read the sensor data into the class instance
    
    // Send Lux Value to Serial Output
    // The maximum count is 65535 (0XFFFF)
    // Converting to Lux (count/1.2) results in a maximum lux value of 54612.
    
    // Convert LuxValue into a string that can be sent over the serial output.
    // Use snprintf to convert LuxValue into a string representation of an 
    // unsigned decimal integer. Add leading zeros. 
    // snprintf adds a trailing null character.
    
    // Buffer to hold 6 bytes, one for each digit, plus terminating null char. 
    char char_buffer[6]; 
    
    // Convert and send LuxBytes for reference.
    // Convert LuxBytes and store value in char_buffer with leading zeros.
    snprintf(char_buffer, 6,"%05u", lux.getLuxByte()); 
    
    for(int i=0;i<5;i++){
        serial.write(char_buffer[i]);
    }
    
    serial.write(' '); // Separate Output on serial terminal.
    
    // Convert LuxValue and store value in char_buffer with leading zeros.
    snprintf(char_buffer,6, "%05u", lux.getLuxValue());

    // Step through the buffer and send each byte via the serial output.
    for(int i=0;i<5;i++){
        serial.write(char_buffer[i]);
    }

    serial.write('\r');
    serial.write('\n');
    
    draw_buffer2(i2c);
    delay_ms(2000);
    
    clear_buffer(i2c);
    delay_ms(1000);
    
    draw_buffer3(i2c);
    delay_ms(2000);
    
    clear_buffer(i2c);
    delay_ms(1000);
    
    };
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

void OLEDSetup(I2C& i2c){
    // Setup the I2C OLED Display    
    i2c.start(OLED_ADDR);
    // Initialisation Based upon Application Note Example
    i2c.write(0x00);       // Send Command Byte Stream
    // --
    i2c.write(0xAE);       // Turn Display Off
    // ---
    i2c.write(0xA8);       // Set Multiplex Ratio
    i2c.write(0x3f);       // 1/64 duty cycle
    // ---
    i2c.write(0xD3);       // Set Display Offset
    i2c.write(0x00);       // No offset applied
    // ---
    i2c.write(0x40);       // Set Display Start Line #0
    // ---
    i2c.write(0xA1);       // Set Segment Remap (Flips Display) A0/A1
    // ---
    i2c.write(0xC8);       // COM Scan Direction c0/c8
    // ---
    i2c.write(0xDA);       // Set COM pins
    i2c.write(0x12);       // 0x12 - See Application Note SW INIT
    // ---
    i2c.write(0x81);       // Set Contrast
    i2c.write(0x7F);       // Default Contrast Level 127/255
    // ---
    i2c.write(0xA4);       // Entire Display On - Output follows RAM Contents
    // ---
    i2c.write(0xA6);       // Set Normal Display 0xA7 inverts
    // ---
    i2c.write(0xD5);       // Display Clk Div
    i2c.write(0x80);       // Default Value 0x80
    // ---
    i2c.write(0x8D);       // Setup Charge Pump - See SSD1306 Application Note
    i2c.write(0x14);       // Enable Charge Pump during display on.
    // ---
    i2c.write(0xD9);       // Setup Precharge
    i2c.write(0x22);
    // ---
    i2c.write(0xDB);       // VCOMH DESELECT
    i2c.write(0x30);
    // ---
    i2c.write(0x20);       // Set Mem Addr Mode
    i2c.write(0x00);       // Horzontal
    // ---
    i2c.write(0xAF);       // Display On
    // ---
    i2c.stop();            // Stop Transmitting
}

void draw_buffer2(I2C& i2c){
    // Draw buffer on display
    i2c.start(OLED_ADDR);
    i2c.write(0x00);    // Control Byte Command Stream
    i2c.write(0x21);    // Setup Column Addresses
    i2c.write(0x00);    // Col Start Addr
    i2c.write(0x7F);    // Col End Addr
    i2c.write(0x22);    // Set Page Addr
    i2c.write(0x00);    // Start Page 0
    i2c.write(0x07);    // End Page 7
    i2c.stop();

    for (uint16_t i=0; i<1024; i++){
        i2c.start(OLED_ADDR);
        i2c.write(0x40);      // Control Byte Data Stream
        for (uint8_t x=0; x<16; x++) {
            i2c.write(rodent[i]);
            i++;
        }
        i--;
        i2c.stop();
    }
}

void draw_buffer3(I2C& i2c){
    // Draw buffer on display
    i2c.start(OLED_ADDR);
    i2c.write(0x00);    // Control Byte Command Stream
    i2c.write(0x21);    // Setup Column Addresses
    i2c.write(0x00);    // Col Start Addr
    i2c.write(0x7F);    // Col End Addr
    i2c.write(0x22);    // Set Page Addr
    i2c.write(0x00);    // Start Page 0
    i2c.write(0x07);    // End Page 7
    i2c.stop();

    for (uint16_t i=0; i<1024; i++){
        i2c.start(OLED_ADDR);
        i2c.write(0x40);      // Control Byte Data Stream
        for (uint8_t x=0; x<16; x++) {
            i2c.write(test_pattern[i]);
            i++;
        }
        i--;
        i2c.stop();
    }
}

void clear_buffer(I2C& i2c){
    // Draw buffer on display
    i2c.start(OLED_ADDR);
    i2c.write(0x00);    // Control Byte Command Stream
    i2c.write(0x21);    // Setup Column Addresses
    i2c.write(0x00);    // Col Start Addr
    i2c.write(0x7F);    // Col End Addr
    i2c.write(0x22);    // Set Page Addr
    i2c.write(0x00);    // Start Page 0
    i2c.write(0x07);    // End Page 7
    i2c.stop();

    for (uint16_t i=0; i<1024; i++){
        i2c.start(OLED_ADDR);
        i2c.write(0x40);      // Control Byte Data Stream
        for (uint8_t x=0; x<16; x++) {
            i2c.write(0x00);
            i++;
        }
        i--;
        i2c.stop();
    }
}
