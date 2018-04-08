// STM32F103 I2C Setup & Test
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>             // Enable fixed width integers
#include <stdio.h>              // Newlib-nano
#include "buffer_class.h"       // Circular Buffer Class
#include "clock.h"              // Setup system and peripheral clocks
#include "serial.h"             // USART1 Setup & Functions
#include "delay.h"              // Simple Delay Function
#include "i2c.h"                // I2C Setup and control functions
#include "displays/rodent_buffer.h"         // Rodent Display Buffer
#include "displays/test_pattern_buffer.h"   // Test Pattern Display Buffer
#include "displays/ascii_buffer.h"          // ASCII Text Buffer
#include "sensors/BH1750FVI.h"              // I2C Lux Sensor
#include "displays/ssd1306.h"               // OLED Display
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
void draw_progress(uint8_t progress, uint8_t steps, SSD1306& oled);
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
    
    SSD1306 oled(i2c,OLED_ADDR);    // Create instance of OLED Display 
    //OLEDSetup(i2c);                 // Create instance of OLED Display
    oled.setup();                   // Setup OLED Display
    
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
    //longdelay(0xFFFF);  // Allow time for reading to be taken, auto power down.
    //longdelay(0xFFFF);  // Allow time for reading to be taken, auto power down.
    delay_ms(1000);
    
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
    
    // Test Code - Attemp to write the Lux measurement to the OLED display
    oled.clear_buffer(); // Clear the buffer to begin with
    oled.setCursor(0,0); 
    
    for(int i=0;i<5;i++){
        serial.write(char_buffer[i]); // Write Character to Serial Port
        oled.write(char_buffer[i], ascii_buffer); // Draw character on OLED
    }
    
    serial.write(' '); // Separate Output on serial terminal.
    oled.write(' ', ascii_buffer);
    
    // Convert LuxValue and store value in char_buffer with leading zeros.
    snprintf(char_buffer,6, "%05u", lux.getLuxValue());

    // Step through the buffer and send each byte via the serial output.
    for(int i=0;i<5;i++){
        serial.write(char_buffer[i]); // Write Char to Serial Port
        oled.write(char_buffer[i], ascii_buffer); // Draw character on OLED
    }

    serial.write('\r');
    serial.write('\n');
    delay_ms(2000); 
    oled.clear_buffer();

    // Progress Bar
    
    // Progress Bar will have 10 steps + start and stop point. Width 12.
    oled.setCursor(0,0);
    draw_progress(0,10,oled);
    oled.setCursor(2,0);
    draw_progress(40,10,oled);
    oled.setCursor(4,0);
    draw_progress(85,10,oled);
    oled.setCursor(6,0);
    draw_progress(100,10,oled);

    delay_ms(2000);
    oled.clear_buffer();
    
    // Test code to draw test patterns to the OLED display.
    oled.drawBuffer(rodent);
    delay_ms(2000);
    
    oled.clear_buffer();
    delay_ms(1000);
    
    oled.drawBuffer(test_pattern);
    delay_ms(2000);
    
    oled.clear_buffer();
    delay_ms(1000);
    
    oled.drawBuffer(ascii_buffer);
    delay_ms(2000);
    
    oled.clear_buffer();
    delay_ms(1000);
    };
}

void draw_progress(uint8_t progress, uint8_t steps, SSD1306& oled){
    uint8_t width = steps +2;
        
    for(int i=0; i<width; i++){
        if(i==0){                               // If Start/End Point
            oled.write(91,ascii_buffer);        // Start Point Character '['
        }else if (i==11){
            oled.write(93,ascii_buffer);        // End Point Character ']'
        }else{
            // Compare progress with step position.
            // If progress is beyond or equal to the step, fill in step.
            // Otherwise show as an empty step.
            if (progress/steps >= i){
                oled.write(127,ascii_buffer);   // Checkered Block
            }else{
                oled.write(45,ascii_buffer);    // '-'
            }
        }
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
