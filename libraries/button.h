// button.h
#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h> // Fixed Width Integers
    
    // BUTTON_MASK - Defines the period of time that is ignored for debouncing.
    //               Assuming 1ms history update allows for 16ms of switch bounce.
    // Override this value by defining before including this header file.
    #ifndef BUTTON_MASK
    #define BUTTON_MASK 0b11111111000000000000000011111111
    #endif
    
uint8_t is_button_up(uint32_t *button_history);
uint8_t is_button_down(uint32_t *button_history);
uint8_t is_button_pressed(uint32_t *button_history);
uint8_t is_button_released(uint32_t *button_history);


#endif
