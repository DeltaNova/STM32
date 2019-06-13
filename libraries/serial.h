// serial.h
#ifndef SERIAL_H
#define SERIAL_H

#include "buffer_class.h"
#include "serial.h"
#include "stm32f103xb.h" // Need as direct reference to HW
//#include "buffer.h"      // Uses circular buffers.

#include <stdint.h> // uint8_t

// Currently this is written for USART1 only
class Serial {
    private:
        Buffer& rxbuffer;
        Buffer& txbuffer;
        
    public:
        Serial(Buffer& rx,Buffer& tx) : rxbuffer(rx), txbuffer(tx)
        {
            // Anything which needs to be executed when class instansiated.    
        }
        
        // Send data without using buffer
        void write(uint8_t);

        // Read from rxbuffer
        uint8_t read();     

        // Sends contents of txbuffer
        void write_buffer(); 
        
        // Adds array contents to txbuffer
        void write_array(uint8_t *array, uint8_t array_length);
        
        // Writes Newline characters (LF & CR)
        void newline();
        
        // Clear line by jumping to the start, overwriting the desired number of
        // of chars with spaces before jumping back to the start of the line. 
        void lineClear(uint8_t numChars2Clear);
    

        /*
        // Currently needs rewrite to declare buffers, causing library to fail compile.

        // SerialReceiveEcho();
        // --------------------
        // Checks if RXNE flag in USART1->SR, read data if set.
        // Echoes data back to sending terminal.
        void echo();
        */

        void setup(); // Setup USART1 & IRQ Priority

};
#endif // SERIAL_H
