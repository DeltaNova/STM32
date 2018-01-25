// STM32F103 I2C Setup & Test
////////////////////////////////////////////////////////////////////////////////
#include "clock.h"          // Setup system and peripheral clocks
#include "buffer.h"         // Circular Buffers
#include "serial.h"         // USART1 Setup & Functions
#include "delay.h"          // Simple Delay Function
#include "i2c.h"            // I2C Setup and control functions
#include <stdint.h>         // uint8_t
#include <stdio.h>          // Newlib-nano 
#include "stm32f103xb.h"    // HW Specific Header
////////////////////////////////////////////////////////////////////////////////
// Function Declarations
extern "C" void USART1_IRQHandler(void);
////////////////////////////////////////////////////////////////////////////////
// Buffers
// -------
// Create buffers - Size defined by buffer.h or variable for compiler.
volatile struct Buffer serial_tx_buffer {{},0,0};
volatile struct Buffer serial_rx_buffer {{},0,0};
volatile struct Buffer i2c_rx_buffer{{},0,0};
////////////////////////////////////////////////////////////////////////////////
// Main - Called by the startup code.
int main(void) {
    ClockSetup();       // Setup System & Peripheral Clocks
    SerialSetup();      // Enable Serial Support - Currently USART1 Specific
    I2CSetup(&i2c_rx_buffer);

    // USART1 Message to confirm program running - Using for Debugging
    uint8_t test_message[] = "Waiting!\n\r"; //Size 10, escape chars 1 byte each
    // Send a message to the terminal to indicate program is running.
    LoadBuffer(&serial_tx_buffer, test_message, 10);
    SerialBufferSend(&serial_tx_buffer);

    while(1){
    delay(1000);
    // Initially need a simple device to allow development of comms functions.
    // Using BH1750FVI Breakout board - Ambient Light Sensor
    I2CWriteMode(0xB8); // Slave Address
    I2CWriteData(0x01); // BH1750FVI - Power On
    I2CStop();          // Required as part of BH1750FVI I2C Comms

    I2CWriteMode(0xB8);
    //I2CWriteData(0x20); // BH1750FVI One Time H-Res Mode.
    I2CWriteData(0x13);   // BH1750FVI Continuous Mode
    I2CStop();            // Required as part of BH1750FVI I2C Comms
    longdelay(0xFFFF);  // Allow time for reading to be taken, auto power down.
    
    // Reads 2 Byte Measurement into i2c_rx_buffer
    //I2CRead2Bytes(0xB9, &i2c_rx_buffer);
    I2CReadData(2, 0xB9, &i2c_rx_buffer);

    SerialBufferSend(&i2c_rx_buffer); // Send measurement via serial.           // TODO: Convert to Lux value before send
    
    SerialSendByte('\r');
    SerialSendByte('\n');
    };
}


