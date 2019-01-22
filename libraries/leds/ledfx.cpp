// ledfx.cpp - WS2812 Effects Library
#include "stm32f103xb.h"        // Need as direct reference to HW
#include "ledfx.h"            // Library Header





void WS2812::RGBLoop(){
  for(int j = 0; j < 3; j++ ) { 
    // Fade IN
    for(int k = 0; k < 256; k++) { 
      switch(j) { 
        case 0: setAllRGB(k,0,0,pixels); break;
        case 1: setAllRGB(0,k,0,pixels); break;
        case 2: setAllRGB(0,0,k,pixels); break;
      }

      writeLED(pixels,NUM_LEDS,DMA_Buffer);
      delay_ms(3);
    }

    // Fade OUT
    for(int k = 255; k >= 0; k--) { 
      switch(j) { 
        case 0: setAllRGB(k,0,0,pixels); break;
        case 1: setAllRGB(0,k,0,pixels); break;
        case 2: setAllRGB(0,0,k,pixels); break;
      }
      writeLED(pixels,NUM_LEDS,DMA_Buffer);
      delay_ms(3);
    }
  }
}

/*
 
void WS2812::CylonBounce(uint8_t R, uint8_t G, uint8_t B, int EyeSize, int SpeedDelay, int ReturnDelay){
  for(int i = 0; i < NUM_LEDS-EyeSize-2; i++) {
    setAllRGB(0,0,0,pixels);
    setPixelRGB(R/10, G/10, B/10, i,pixels);
    for(int j = 1; j <= EyeSize; j++) {
      setPixelRGB(R, G, B, i+j, pixels); 
    }
    setPixelRGB(R/10, G/10, B/10, i+EyeSize+1, pixels);
    writeLED(pixels,NUM_LEDS, DMA_Buffer);
    delay_ms(SpeedDelay);
  }
  delay_ms(ReturnDelay);
  for(int i = NUM_LEDS-EyeSize-2; i > 0; i--) {
    setAllRGB(0,0,0,pixels);
    setPixelRGB(R/10, G/10, B/10,i,pixels);
    for(int j = 1; j <= EyeSize; j++) {
      setPixelRGB(R, G, B, i+j, pixels); 
    }
    setPixelRGB(R/10, G/10, B/10,i+EyeSize+1, pixels);
    writeLED(pixels,NUM_LEDS, DMA_Buffer);
    delay_ms(SpeedDelay);
  }
  delay_ms(ReturnDelay);

}
void WS2812::Sparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint8_t SpeedDelay) {

  uint8_t Pixel = getRandomNumber(0,NUM_LEDS);
  setPixelRGB(R,G,B,Pixel,array);
  writeLED(array,NUM_LEDS,DMA_Buffer);
  delay_ms(SpeedDelay);
  setPixelRGB(0,0,0,Pixel,array);
}


void WS2812::SnowSparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint16_t SparkleDelay, uint16_t SpeedDelay) {
    
    // SparkleDelay - Delay Time in ms (0-65535)
    //SpeedDelay   - Delay Time in ms (0-65535)
    //REQ: NUM_LEDS <= 255
    //
    
    // Compile Time Check for global NUM_LED value 
    static_assert(NUM_LEDS > 0, "SnowSparkle - NUM_LEDS needs to be > 0");
    static_assert(NUM_LEDS <= 255, "SnowSparkle - NUM_LEDS needs to be <= 255");
    
    setAllRGB(R,G,B,array);             // Fill LED Array with specified colour.
    uint8_t Pixel = getRandomNumber(0,NUM_LEDS);    // Pick an LED at random
  
    // Change the colour of the selected for a short ammount of time.
    setPixelRGB(0xff,0xff,0xff, Pixel,array); // Set LED Colour
    writeLED(array,NUM_LEDS,DMA_Buffer);      // Update ALL LEDs
    delay_ms(SparkleDelay);                   // Hold ALL LED colours
  
    setPixelRGB(R,G,B,Pixel,array);           // Set ALL LEDs to initial value
    writeLED(array,NUM_LEDS,DMA_Buffer);      // Update ALL LEDs.
    delay_ms(SpeedDelay);                     // Hold ALL LED colours.
}

void WS2812::colorWipe(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint16_t SpeedDelay) {
    
    // SpeedDelay   - Delay Time in ms (0-65535)
    //REQ: NUM_LEDS <= 255
    //
    //SpeedDelay controls how long to wait before changing colour of next LED
    
     
    // Compile Time Check for global NUM_LED value 
    static_assert(NUM_LEDS > 0, "colorWipe - NUM_LEDS needs to be > 0");
    static_assert(NUM_LEDS <= 255, "colorWipe - NUM_LEDS needs to be <= 255");
    
    uint8_t i;
    for(i=0; i<NUM_LEDS; i++) {             // For each LED in turn
      setPixelRGB(R, G, B, i, array);       // Set the new colour of the LED
      writeLED(array,NUM_LEDS,DMA_Buffer);  // Update ALL LED colours
      delay_ms(SpeedDelay);                 // Wait before next LED
  }
}
// TODO: ADD FadeInOut
// TODO: ADD rainbow cycle effect

// TODO@ ADD Bouncing Balls
// TODO: ADD Multi Colour Bouncing Balls


void WS2812::Fire(int Cooling, int Sparking, int SpeedDelay) {
  static uint8_t heat[NUM_LEDS];
  int cooldown;
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUM_LEDS; i++) {
    cooldown = getRandomNumber(0, ((Cooling * 10) / NUM_LEDS) + 2);
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }

  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( getRandomNumber(0,255) < Sparking ) {
    int y = getRandomNumber(0,7);
    heat[y] = heat[y] + getRandomNumber(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUM_LEDS; j++) {
    setPixelHeatColor(j, heat[j] );
  }
  writeLED(pixels,NUM_LEDS,DMA_Buffer);
  
  delay_ms(SpeedDelay);
}

void WS2812::setPixelHeatColor (uint8_t Pixel, uint8_t temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  uint8_t t192 = round((temperature/255.0)*191);
  // calculate ramp up from
  uint8_t heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252

  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixelRGB(255, 255, heatramp, Pixel, pixels);
  } else if( t192 > 0x40 ) {             // middle
    setPixelRGB(255, heatramp, 0, Pixel, pixels);
  } else {                               // coolest
    setPixelRGB(heatramp, 0, 0, Pixel, pixels);
  }
}


void WS2812::theaterChase(uint8_t R, uint8_t G, uint8_t B, uint8_t SpeedDelay) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < NUM_LEDS; i=i+3) {
        setPixelRGB(R,G,B,i+q,pixels);    //turn every third pixel on
      }
      writeLED(pixels,NUM_LEDS,DMA_Buffer);
      delay_ms(SpeedDelay);
      for (int i=0; i < NUM_LEDS; i=i+3) {
        setPixelRGB(0,0,0,i+q,pixels);        //turn every third pixel off
      }
    }
  }
}

void WS2812::theaterChaseRainbow(int SpeedDelay) {
    // Lower Delay values work best, a value of 1-4 seems best/smoothest
    // A delay of 0 doesnt seem to work.
  uint8_t *c;
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
        for (int i=0; i < NUM_LEDS; i=i+3) {
          c = Wheel( (i+j) % 255);
          setPixelRGB(*c, *(c+1), *(c+2),i+q,pixels);    //turn every third pixel on
        }
        writeLED(pixels,NUM_LEDS, DMA_Buffer);
        delay_ms(SpeedDelay);

        for (int i=0; i < NUM_LEDS; i=i+3) {
          setPixelRGB( 0,0,0,i+q,pixels);        //turn every third pixel off
        }
    }
  }
}

uint8_t* WS2812::Wheel(uint8_t WheelPos) {
  static uint8_t c[3];
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }
  return c;
}

void WS2812::meteorRain(uint8_t R, uint8_t G, uint8_t B, uint8_t meteorSize, uint8_t meteorTrailDecay, bool meteorRandomDecay, int SpeedDelay) {  
    setAllRGB(0,0,0,pixels);
    for(int i = 0; i < NUM_LEDS+NUM_LEDS; i++) {
        // fade brightness all LEDs one step
        for(int j=0; j<NUM_LEDS; j++) {
            if( (!meteorRandomDecay) || (getRandomNumber(0,10)>5) ) {
                fadeToBlack(j, meteorTrailDecay );        
            }
        }

        // draw meteor
        for(int j = 0; j < meteorSize; j++) {
            if( ( i-j <NUM_LEDS) && (i-j>=0) ) {
                setPixelRGB(R, G, B, i-j, pixels);
            } 
        }
        writeLED(pixels,NUM_LEDS, DMA_Buffer);
        delay_ms(SpeedDelay);
    }
}

void WS2812::fadeToBlack(int ledNo, uint8_t fadeValue) {
    // Used by meteorRain()
    uint8_t r, g, b;
    r = pixels[ledNo][0];
    g = pixels[ledNo][2];
    b = pixels[ledNo][1];

    r=(r<=10)? 0 : (int) r-(r*fadeValue/256);
    g=(g<=10)? 0 : (int) g-(g*fadeValue/256);
    b=(b<=10)? 0 : (int) b-(b*fadeValue/256);
    
    setPixelRGB(r,g,b,ledNo,pixels);
}

void WS2812::RunningLights(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3],  uint16_t WaveDelay) {
    
    // Compile Time Check for global NUM_LED value 
    static_assert(NUM_LEDS > 0, "RunningLights - NUM_LEDS needs to be > 0");
    static_assert(NUM_LEDS <= 255, "RunningLights - NUM_LEDS needs to be <= 255");
    
    uint8_t Position = 0;
    uint16_t j; // j needs to be large enough to hold MAX NUM_LED, 255*2 = 510.
    for(j=0; j<NUM_LEDS*2; j++){
        Position++; // = 0; //Position + Rate;
        for(uint8_t i=0; i<NUM_LEDS; i++) {
            setPixelRGB(((sin(i+Position) * 127 + 128)/255)*R,
                    ((sin(i+Position) * 127 + 128)/255)*G,
                    ((sin(i+Position) * 127 + 128)/255)*B,
                    i,pixels);
        }
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(WaveDelay);
    }
}

void WS2812::Strobe(uint8_t R, uint8_t G, uint8_t B, uint8_t StrobeCount, uint16_t FlashDelay, uint16_t EndPause){

  for(int j = 0; j < StrobeCount; j++) {

    setAllRGB(R,G,B,pixels);
    writeLED(pixels,NUM_LEDS, DMA_Buffer);
    delay_ms(FlashDelay);
    setAllRGB(0,0,0,pixels);
    writeLED(pixels,NUM_LEDS, DMA_Buffer);
    delay_ms(FlashDelay);
  }
 delay_ms(EndPause);
}

void WS2812::Twinkle(uint8_t R, uint8_t G, uint8_t B, uint8_t Count, uint8_t SpeedDelay, bool OnlyOne) {
  setAllRGB(0,0,0,pixels);
  for (uint8_t i=0; i<Count; i++) {
     setPixelRGB(R,G,B,getRandomNumber(0,NUM_LEDS),pixels);
     writeLED(pixels,NUM_LEDS, DMA_Buffer);
     delay_ms(SpeedDelay);
     if(OnlyOne) { 
       setAllRGB(0,0,0,pixels); 
     }
   }
  delay_ms(SpeedDelay);
}

void WS2812::TwinkleRandom(uint8_t Count, uint8_t SpeedDelay, bool OnlyOne) {
  setAllRGB(0,0,0,pixels);
  for (uint8_t i=0; i<Count; i++) {
     setPixelRGB(getRandomNumber(0,255),getRandomNumber(0,255),getRandomNumber(0,255),getRandomNumber(0,NUM_LEDS),pixels);
     writeLED(pixels,NUM_LEDS, DMA_Buffer);
     delay_ms(SpeedDelay);
     if(OnlyOne) { 
       setAllRGB(0,0,0,pixels); 
     }
   }
  delay_ms(SpeedDelay);
}
*/
