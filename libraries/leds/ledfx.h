// ledfx.h - WS2812 Effects Library 

#include "stm32f103xb.h"    // Need as direct reference to HW

#ifndef LEDFX_H
#define LEDFX_H

class WS2812 {
    private:    // Can only be accessed by fuctions in this class.
        uint8_t NUM_LEDS{0}; // Uniform Initilisation 
        
        // Create the array to hold the pixel data. Placing this inside the 
        // class since additional instances will each require a unique array.
        uint8_t pixels[NUM_LEDS][3]= {0};
    public:     // Can be accessed by any function in program.
        // Default Constructor
        WS2812(uint8_t LED_COUNT) : NUM_LEDS{LED_COUNT}
        {
            // Anything which needs to be executed when class instansiated.
        }

    void RGBLoop();
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















