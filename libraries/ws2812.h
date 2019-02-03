// ws2812.h
// Header File for ws2812 library

// Template functions created to allow use with LED strings of different lengths within the same program.
// Templated functions will cause a slight increase in code size dependant on how many times they are required.
// DMA Buffer passed as a variable since different strings might use different DMA channels in future.
#include "systick.h"              // SysTick Delay Function



#ifndef WS2812_H
#define WS2812_H
template <uint8_t LEDS>
void setAllRGB(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[LEDS][3]){
    // R,G,B are individual colour values.
    // array is the array which holds the data for all the pixels in the string.
    for (uint8_t i = 0; i < LEDS; i++){
            array[i][0] = R;
            array[i][1] = G;
            array[i][2] = B;   
    }
}

template <uint8_t LEDS>
void RGBLoop(uint8_t (&array)[LEDS][3], uint8_t (&Buffer)[2*BYTES_PER_LED]){
  for(int j = 0; j < 3; j++ ) { 
    // Fade IN
    for(int k = 0; k < 256; k++) { 
      switch(j) { 
        case 0: setAllRGB(k,0,0,array); break;
        case 1: setAllRGB(0,k,0,array); break;
        case 2: setAllRGB(0,0,k,array); break;
      }

      writeLED(array,LEDS,Buffer);
      delay_ms(3);
    }

    // Fade OUT
    for(int k = 255; k >= 0; k--) { 
      switch(j) { 
        case 0: setAllRGB(k,0,0,array); break;
        case 1: setAllRGB(0,k,0,array); break;
        case 2: setAllRGB(0,0,k,array); break;
      }
      writeLED(array,LEDS,Buffer);
      delay_ms(3);
    }
  }
}
#endif // WS2812_H
