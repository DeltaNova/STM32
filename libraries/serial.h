// serial.h
#ifndef SERIAL_H
#define SERIAL_H

#include "serial.h"
#include "stm32f103xb.h" // Need as direct reference to HW
#include "buffer.h"      // Uses circular buffers.
#include <stdint.h> // uint8_t

// Currently this is written for USART1 only
class Serial {
    private:
    public:
        Serial(){}
        
        void write(uint8_t);

        uint8_t read();

        void write_buffer(volatile struct Buffer *serial_tx_buffer);

        void write_array(uint8_t *array, uint8_t array_length);


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
