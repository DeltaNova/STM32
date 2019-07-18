// button.h
#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>             // Fixed Width Integers
#include "stm32f103xb.h"        // HW Specific Header STM32F103

    
    // BUTTON_MASK - Defines the period of time that is ignored for debouncing.
    //               Assuming 1ms history update allows for 16ms of switch bounce.
    // Override this value by defining before including this header file.
    #ifndef BUTTON_MASK
    #define BUTTON_MASK 0b11111111000000000000000011111111
    #endif

// Functions to test for button state based on the recorded history.    
uint8_t is_button_up(uint32_t *button_history);
uint8_t is_button_down(uint32_t *button_history);
uint8_t is_button_pressed(uint32_t *button_history);
uint8_t is_button_released(uint32_t *button_history);

// Define a function pointer for the reading of a button. It will be used to
// to allow different read functions to be used by update_button()
typedef uint32_t (*readButtonFcn)();

// Button Debounce 
void update_button(uint32_t *button_history, readButtonFcn read_button);

enum class pressType{
    IDLE,       // No Press Event
    SHORT,      // Short Press
    LONG,       // Long Press
    VLONG,      // Very Long Press
};
struct Button{
    // Store the SysTick Counter Value on press and release to enable
    // calculation of press duration.
    uint32_t pressStart = 0;
    uint32_t pressStop = 0;
    // The following values are based off a 1ms clock tick.
    uint16_t shortPressMax = 1000;  // 1 Second
    uint16_t longPressMax = 5000;   // 5 Seconds
};

pressType buttonAction(uint32_t *button_history, Button &button, uint32_t count);


////////////////////////////////////////////////////////////////////////////////
// Hardware Specific Functions
// STM32F103 - Requires '#include "stm32f103xb.h"'
uint32_t readButtonPB6();


#endif
