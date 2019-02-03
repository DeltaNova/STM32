// ledfx.h - WS2812 Effects Library 

#include "stm32f103xb.h"    // Need as direct reference to HW

#ifndef LEDFX_H
#define LEDFX_H


template <uint8_t NUM_LEDS>
class WS2812 {
    private:
        uint8_t NUMLEDS = NUM_LEDS;
        uint8_t ARRAY[NUM_LEDS][3];
        uint8_t BYTES_PER_LED = 24;
    public:
        WS2812()
        {
            // Nothing to do here
        }
    
    void setPixelRGB(uint8_t R, uint8_t G, uint8_t B, uint8_t pixel);
}

template <uint8_t NUM_LEDS>
void WS2812<NUM_LEDS>::setPixelRGB(uint8_t R, uint8_t G, uint8_t B, uint8_t pixel){
    // pixel is the LED position in the string
    // R,G,B are individual colour values.
    // ARRAY is the array which holds the data for all the pixels in the string.
    ARRAY[pixel][0] = R;
    ARRAY[pixel][1] = G;
    ARRAY[pixel][2] = B;
}

/*
void WS2812::RGBLoop(){
  for(int j = 0; j < 3; j++ ) { 
    // Fade IN
    for(int k = 0; k < 256; k++) { 
      switch(j) { 
        case 0: setAllRGB(k,0,0,array); break;
        case 1: setAllRGB(0,k,0,array); break;
        case 2: setAllRGB(0,0,k,array); break;
      }

      writeLED(array,NUM_LEDS,DMA_Buffer);
      delay_ms(3);
    }

    // Fade OUT
    for(int k = 255; k >= 0; k--) { 
      switch(j) { 
        case 0: setAllRGB(k,0,0,array); break;
        case 1: setAllRGB(0,k,0,array); break;
        case 2: setAllRGB(0,0,k,array); break;
      }
      writeLED(array,NUM_LEDS,DMA_Buffer);
      delay_ms(3);
    }
  }
}
*/

//void Sparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint8_t SpeedDelay) {
//
//  uint8_t Pixel = getRandomNumber(0,NUM_LEDS);
//  setPixelRGB(R,G,B,Pixel,array);
//  writeLED(array,NUM_LEDS,DMA_Buffer);
//  delay_ms(SpeedDelay);
//  setPixelRGB(0,0,0,Pixel,array);
//}






//class WS2812 {
//    private:    // Can only be accessed by fuctions in this class.
//        uint8_t NUM_LEDS{0}; // Uniform Initilisation 
        
        // Create a reference tp the array that will hold the pixel data. 
        // Passing by reference as arrays will be different lengths based on 
        //the number of LEDS in each string.
//        uint8_t & array;
        // DEV NOTE: A fixed length array could be defined but it would need to
        // be the size of the longest string. This would mean a short string 
        // would be allocated a large array. This would be result in wasted 
        // space, especially if working with very long and very short stings of 
        // LEDS. 
        // Passing the array by reference means only the space required by each 
        // array is allocated.
        
        
//    public:     // Can be accessed by any function in program.
        // Default Constructor
//        WS2812(uint8_t & LED_ARRAY, uint8_t LED_COUNT) : NUM_LEDS{LED_COUNT}, array{LED_ARRAY}
//        {
            // Anything which needs to be executed when class instansiated.
//        }

//    void RGBLoop();
    /*
    void Sparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint8_t SpeedDelay);
    void RunningLights(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3],  uint16_t WaveDelay);
    void SnowSparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint16_t SparkleDelay, uint16_t SpeedDelay);   // Updated
    void colorWipe(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint16_t SpeedDelay);                            // Updated
    void CylonBounce(uint8_t R, uint8_t G, uint8_t B, int EyeSize, int SpeedDelay, int ReturnDelay);
    void Strobe(uint8_t R, uint8_t G, uint8_t B, uint8_t StrobeCount, uint16_t FlashDelay, uint16_t EndPause);
    void Twinkle(uint8_t R, uint8_t G, uint8_t B, uint8_t Count, uint8_t SpeedDelay, bool OnlyOne);
    void TwinkleRandom(uint8_t Count, uint8_t SpeedDelay, bool OnlyOne);
    void theaterChase(uint8_t R, uint8_t G, uint8_t B, uint8_t SpeedDelay);
    void meteorRain(uint8_t R, uint8_t G, uint8_t B, uint8_t meteorSize, uint8_t meteorTrailDecay, bool meteorRandomDecay, int SpeedDelay);
    void fadeToBlack(int ledNo, uint8_t fadeValue);
    void theaterChaseRainbow(int SpeedDelay);
    void Fire(int Cooling, int Sparking, int SpeedDelay);

    uint8_t* Wheel(uint8_t WheelPos);
    void setPixelHeatColor (uint8_t Pixel, uint8_t temperature);
    */
};

#endif // LEDFX_H















