// ws2812.h - Header File for ws2812 library
#ifndef WS2812_H
#define WS2812_H
#include "stm32f103xb.h"        // HW Specific Header

#define NUM_LEDS 5
#define BYTES_PER_LED 24
////////////////////////////////////////////////////////////////////////////////
// WS2812 Function Declarations

void DMA_Setup();
void PWM_Setup();
extern "C" void DMA1_Channel5_IRQHandler();


void setAllRGB(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3]);

//void DMA1_Channel5_IRQHandler();

void loadColour(uint8_t *colour, uint8_t *array, uint8_t offset);
void loadReset(uint8_t *array, uint8_t offset);
void writeLED(uint8_t (*colour)[3], uint8_t length, uint8_t *buffer);

////////////////////////////////////////////////////////////////////////////////
// WS2812 Variables
// Used by DMA1_Channel5_IRQHandler & writeLED
extern uint8_t currentLED;      // Tracks LED write progress

// Points to the colour sequence being sent. Used to allow DMA_ISR to load
// data into buffer.
extern uint8_t (*LEDSequence)[3];



extern uint8_t LED_COUNT;      // Number of LEDs in string.

// The DMA Buffer needs to be able to hold the data for 2 LEDs. When the data
// for one LED is sent the DMA HT (Half Transfer) Flag is set. After the data 
// for the next LED is send the DMA TC (Transfer Complete) Flag is set. 
// The DMA transfer is setup for cicular mode and will then wrap around to the
// start.

extern uint8_t DMA_Buffer[2*BYTES_PER_LED];
extern uint8_t pixels[NUM_LEDS][3];    // Array of LED Data



#endif //WS2812_H
