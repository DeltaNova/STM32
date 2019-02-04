// ledfx.h
// Header File for ws2812 library

// Template functions created to allow use with LED strings of different lengths within the same program.
// Templated functions will cause a slight increase in code size dependant on how many times they are required.
// DMA Buffer passed as a variable since different strings might use different DMA channels in future.
#include "systick.h"              // SysTick Delay Function



#ifndef WS2812_H
#define WS2812_H

static uint8_t currentLED = 0;      // Tracks LED write progress
// Points to the colour sequence being sent. Used to allow DMA_ISR to load
// data into buffer.
static uint8_t (*LEDSequence)[3];

void loadReset(uint8_t *array, uint8_t offset);
void loadColour(uint8_t *colour, uint8_t *array, uint8_t offset);
void writeLED(uint8_t (*colour)[3], uint8_t length, uint8_t *buffer);

template <uint8_t LEDS>
void setPixelRGB(uint8_t R, uint8_t G, uint8_t B, uint8_t pixel, uint8_t (&array)[LEDS][3]){
    // pixel is the LED position in the string
    // R,G,B are individual colour values.
    // array is the array which holds the data for all the pixels in the string.
    array[pixel][0] = R;
    array[pixel][1] = G;
    array[pixel][2] = B;
    }

template <uint8_t LEDS>
void setPixel(uint8_t colour[3], uint8_t pixel, uint8_t (&array)[LEDS][3]){
    // pixel is the LED position in the string
    // colour is an array of R,G,B values
    // array is the array which holds the data for all the pixels in the string.
    array[pixel][0] = colour[0];
    array[pixel][1] = colour[1];
    array[pixel][2] = colour[2];
    }

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
void setAll(uint8_t colour[3], uint8_t (&array)[LEDS][3]){
    // R,G,B are individual colour values.
    // array is the array which holds the data for all the pixels in the string.
    for (uint8_t i = 0; i < LEDS; i++){
            array[i][0] = colour[0];
            array[i][1] = colour[1];
            array[i][2] = colour[2];   
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

template <uint8_t LEDS>
void colorWipe(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[LEDS][3], uint16_t SpeedDelay, uint8_t (&Buffer)[2*BYTES_PER_LED]) {
     // SpeedDelay   - Delay Time in ms (0-65535)
     //REQ: NUM_LEDS <= 255
     //
     //SpeedDelay controls how long to wait before changing colour of next LED
     //
     
    // Compile Time Check for global NUM_LED value 
    static_assert(LEDS > 0, "colorWipe - LEDS needs to be > 0");
    static_assert(LEDS <= 255, "colorWipe - LEDS needs to be <= 255");
    
    uint8_t i;
    for(i=0; i<LEDS; i++) {                 // For each LED in turn
      setPixelRGB(R, G, B, i, array);       // Set the new colour of the LED
      writeLED(array,LEDS,Buffer);  // Update ALL LED colours
      delay_ms(SpeedDelay);                 // Wait before next LED
  }
}

void writeLED(uint8_t (*colour)[3], uint8_t length, uint8_t *buffer){
    /*
    // Setup the transfer of colour information to the LEDS.
    // This function loads the initial information into the array buffer and
    // tracks the progress using the global currentLED variable.
    // The transfer is started. The data that initially isn't within the
    // buffer is loaded later when the DMA HT/TC interrups trigger.
    */
    
    
    if (length <1){
        // No data to send. Return without doing anything else.
        return; 
    }
    
    // Check for exisiting write (Timer and DMA Enabled).
    // Continue when previous write has finished.
    while((TIM2->CR1 & 0x0001) && (DMA1_Channel5->CCR & 0x00000001)){
        // Wait until previous write has finished
    }
    
    // Store the sequence being sent so it can be referenced by the ISR.
    LEDSequence = colour; 
    
    currentLED = 0; // Reset Global variable
    
    if (currentLED < length){
        // Load the colour data into the DMA Buffer (1st Half)
        loadColour(LEDSequence[currentLED], buffer, 0);
    }else{
        loadReset(buffer,0);
    }
    
    currentLED++; // Next LED
    
    if (currentLED < length){
        // Load the colour data into the DMA Buffer (2nd Half)
        loadColour(LEDSequence[currentLED], buffer, BYTES_PER_LED);
    }else{
        loadReset(buffer,BYTES_PER_LED);
    }
    
    currentLED++; // Next LED
    
    // CNDTR is size of buffer to transfer NOT the size of the data to transfer.
    DMA1_Channel5->CNDTR = (2*BYTES_PER_LED);  // Set Buffer Size
    DMA1_Channel5->CCR |= 0x00000001;           // Enable DMA
    TIM2->CR1 |= 0x0001;                        // Enable Timer
}
void loadColour(uint8_t *colour, uint8_t *array, uint8_t offset){
    // Load a colour into an array. An offset is provided to enable
    // multiple colours to be loaded into the same array at differnt points.
    uint8_t i;
    /*
    // colour is an RGB Array
    // colour[0] = RED Component
    // colour[1] = GREEN Component
    // colour[2] = BLUE Component
    // Order for output array is GRB
    */
    for(i=0; i<8; i++){ // Load GREEN Component
        array[i+offset] = ((colour[1]<<i) & 0x80) ? 0x0F:0x09;
    }
    for(i=0; i<8; i++){ // Load RED Component (Offset by 8 bits from GREEN)
        array[i+offset+8] = ((colour[0]<<i) & 0x80) ? 0x0F:0x09;
    }
    for(i=0; i<8; i++){ // Load BLUE Component (Offset by 16 bits from GREEN)
        array[i+offset+16] = ((colour[2]<<i) & 0x80) ? 0x0F:0x09;
    }
   
}
void loadReset(uint8_t *array, uint8_t offset){
    // Load zeros into buffer to generate PWM reset period.
    uint8_t i;
    for (i=0; i<BYTES_PER_LED; i++){
        array[i+offset] = 0x00;    
    }    
}    

#endif // WS2812_H
