// button.cpp
#include "button.h"

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


void update_button(uint32_t *button_history, readButtonFcn read_button){ 
    // Function called by Systick every 1ms
    // Parameters
    // button_history   - What the button has been doing
    //                  - Value passed will relate to a particular button.
    // read_button      - Reads the button status from hardware.
    //                  - Function passed will read a particular button.
    // Bit shift button history to make room for new reading.
    *button_history = *button_history << 1;
    // Read current button state into history.
    *button_history |= read_button();
}

pressType buttonAction(uint32_t *button_history, Button &button, uint32_t count){ 
    // Button Action from Polling - Works out if and for how long button pressed.
    // Parameters:
    // button_history - Record of the press status over time
    // button         - Struct holding parameters relating to the button.
    // count          - count value of upwards only counter to time press length.
    pressType press = pressType::IDLE;
    uint32_t buttondelta;
    
    if (is_button_pressed(button_history)){
        // Reset Press Duration Counters to current counter value
        button.pressStart = count;
        button.pressStop = count;
    }
    
    if (is_button_released(button_history)){
        button.pressStop = count;
        
        if (button.pressStop < button.pressStart){
            // Counter has rolled over (at least once, multi rollover not handled)
            buttondelta = ((0xFFFFFFFF - button.pressStart) + 1 + button.pressStop);
        }else{
            buttondelta = (button.pressStop - button.pressStart);
        }

        // Check press time and select the message to load into the buffer.
        if (buttondelta < button.shortPressMax){        // Short Press
            press = pressType::SHORT;
        }else if (buttondelta < button.longPressMax){   // Long Press
            press = pressType::LONG;
        }else{                                          // Very Long Press
            press = pressType::VLONG;
        }       
    }
    return press;
}
////////////////////////////////////////////////////////////////////////////////
// Hardware Specific Functions
// STM32F103

uint32_t readButtonPB6(){
    // Read the button state - Return 1 for pressed, 0 for released.
    uint32_t button_state;
    if (GPIOB->IDR & 0x00000040){   // PB6 High (Not Pressed)
        button_state = 0;
    }else{                          // PB6 Low (Pressed)
        button_state = 1;
    }
    return button_state;
}
