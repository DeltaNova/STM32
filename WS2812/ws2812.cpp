// ws2812.cpp
// Rewrite of I2CTest Code to create the appearance of concurrent operation of 
// the stepper motor, oled display and sensor readings without an RTOS. 
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>             // Enable fixed width integers
#include <stdio.h>              // Newlib-nano
#include <stdlib.h>             // rand()
#include <math.h>               // sin()
#include "buffer_class.h"       // Circular Buffer Class
#include "clock.h"              // Setup system and peripheral clocks
//#include "delay.h"              // Simple Delay Function
#include "stm32f103xb.h"        // HW Specific Header
#include "systick.h"            // SysTick Configuration

#ifndef BYTES_PER_LED
// Override the value in ws2812.h (Default: 24)
#define BYTES_PER_LED 24 // Number of bytes holding colour data for each LED.
#endif  // BYTES_PER_LED
#include "ledfx.h"     // WS2812 LED Library

extern volatile uint32_t ticks; // SysTick Library
///////////////////////////////////////////////////////////////////////////////
// Buffers
// -------
// Create buffers - Size defined by buffer_class.h or variable for compiler.
//Buffer serial_rx;
//Buffer serial_tx;
//Buffer rx_buffer;
////////////////////////////////////////////////////////////////////////////////
// Defines
// -------
// LED Definitions

#define NUM_LEDS 5

uint8_t LED_COUNT = NUM_LEDS;      // Number of LEDs in string.

// RGB Colour Definitions - Reduced Brightness (Still very bright)
uint8_t RED2[] = {63,0,0};

#define RED     {63,0,0}
#define GREEN   {0,63,0}
#define BLUE    {0,0,63}
#define WHITE   {63,63,63}
#define PINK    {255,20,147}
#define OFF     {0,0,0}
#define BRIGHTWHITE {255,255,255}

static uint8_t pixels[NUM_LEDS][3]= {0};
static uint8_t pixels2[2][3]={0}; // 2 Pixel Array

////////////////////////////////////////////////////////////////////////////////
// Global Variables

volatile uint32_t flash = 0;        // Used for PC13 LED Flash Toggle Interval


uint8_t colour_rotation = 0;         // Used in loading colour sequence.


static uint32_t counter; // Holds a ms countdown value
// The DMA Buffer needs to be able to hold the data for 2 LEDs. When the data
// for one LED is sent the DMA HT (Half Transfer) Flag is set. After the data 
// for the next LED is send the DMA TC (Transfer Complete) Flag is set. 
// The DMA transfer is setup for cicular mode and will then wrap around to the
// start.

uint8_t DMA_Buffer[2*BYTES_PER_LED] = {};
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Function Declarations
extern "C" void SysTick_Handler(void);
//extern "C" void TIM2_IRQHandler(void);
extern "C" void DMA1_Channel5_IRQHandler(void);

void toggleLed();
//void changeColour();
void PWM_Setup();      // Timer 2 PWM - Ch1 & Ch2
void DMA_Setup();
void Timebase_Setup(); // Timebase from Timer using interrupts
void PC13_LED_Setup(); // Setup PC13 for output LED

//void setPixel(uint8_t colour[3], uint8_t pixel, uint8_t (&array)[NUM_LEDS][3]);

//void setAll(uint8_t colour[3], uint8_t (&array)[NUM_LEDS][3]);








/*
// Effects
void Sparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint8_t SpeedDelay);
void RunningLights(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3],  uint16_t WaveDelay);
void SnowSparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint16_t SparkleDelay, uint16_t SpeedDelay);   // Updated
void CylonBounce(uint8_t R, uint8_t G, uint8_t B, int EyeSize, int SpeedDelay, int ReturnDelay);
void Strobe(uint8_t R, uint8_t G, uint8_t B, uint8_t StrobeCount, uint16_t FlashDelay, uint16_t EndPause);
void Twinkle(uint8_t R, uint8_t G, uint8_t B, uint8_t Count, uint8_t SpeedDelay, bool OnlyOne);
void TwinkleRandom(uint8_t Count, uint8_t SpeedDelay, bool OnlyOne);
void theaterChase(uint8_t R, uint8_t G, uint8_t B, uint8_t SpeedDelay);
void meteorRain(uint8_t R, uint8_t G, uint8_t B, uint8_t meteorSize, uint8_t meteorTrailDecay, bool meteorRandomDecay, int SpeedDelay);
void fadeToBlack(int ledNo, uint8_t fadeValue);

void theaterChaseRainbow(int SpeedDelay);
uint8_t* Wheel(uint8_t WheelPos);

void setPixelHeatColor (uint8_t Pixel, uint8_t temperature);
void Fire(int Cooling, int Sparking, int SpeedDelay);
void ChristmasLightsStart();
void ChristmasLights();
*/
// Main - Called by the startup code.
int main(void) {
    ClockSetup();       // Setup System & Peripheral Clocks
    SysTick_Init();     // Enable SysTick
    PC13_LED_Setup();   // Setup PC13 for output LED
    DMA_Setup();
    PWM_Setup();

    // The counter will count to the reload value where upon it will toggle the
    // output state of each channel back to its reset value. 
    // During the count, before the reload value is reached, the count is 
    // compared with each channels compare value. If the compare value is
    // matched the output state of the matched channel will be toggled.
    
    // Timer 2 Channel 1 Compare Value
    //TIM2->CCR1 = 0x000F;        // 15 (Logic 1)
    TIM2->CCR1 = 0x0000; 
    // Timer 2 Channel 2 Compare Value
    TIM2->CCR2 = 0x0009;        // 9 (Logic 0)
    //delay_ms(2000);
    //ChristmasLightsStart();
    while(1){
        // Triggers Every Second
        toggleLed();    // Toggle LED (PC13)  to indicate loop operational
        
        // Triggers Every 6 Seconds
        //changeColour(); // Change the colours of the WS2812B LEDS
        
        /* Not Working as expected with more than 5 LEDS
        // Green with single reg strobe. Delay added to slow update rate
        writeLED(colour3, 5, DMA_Buffer);
        delay_ms(1000);
        writeLED(colour4, 5, DMA_Buffer);
        delay_ms(1000);
        writeLED(colour5, 5, DMA_Buffer);
        delay_ms(1000);
        writeLED(colour6, 5, DMA_Buffer);
        delay_ms(1000);
        writeLED(colour7, 5, DMA_Buffer);
        delay_ms(1000);
        writeLED(colour8, 5, DMA_Buffer);
        delay_ms(1000);
        writeLED(colour9, 5, DMA_Buffer);
        delay_ms(1000);
        */
        
        
        //setAllRGB(0,0,0,pixels);
        
        //setPixel(RED2,0,pixels);
        
        
        //setPixelRGB(0,0,63,1,pixels);
        //setPixelRGB(255,20,147,2,pixels);
        //writeLED(pixels,NUM_LEDS, DMA_Buffer);
        //delay_ms(1000);
        
        //setAllRGB(138,43,226, pixels);
        //writeLED(pixels,NUM_LEDS, DMA_Buffer);
        //delay_ms(1000);
        
        //setAllRGB(255,255,255, pixels);
        //writeLED(pixels,NUM_LEDS, DMA_Buffer);
        //delay_ms(1000);
        
        //setAllRGB(250,20,147, pixels);
        //writeLED(pixels,NUM_LEDS, DMA_Buffer);
        //delay_ms(4000);
        
        //setAll(RED2, pixels);
        //writeLED(pixels,NUM_LEDS, DMA_Buffer);
        //delay_ms(1000);
        
        //RGBLoop(pixels, DMA_Buffer);
        RGBLoop(pixels2, DMA_Buffer);
        //RGBLoop(pixels2, DMA_Buffer);
        //Sparkle(255,255,255,pixels,3); // White Sparkle
        //Sparkle(getRandomNumber(0,255),getRandomNumber(0,255),getRandomNumber(0,255),pixels,3);
        //CylonBounce(255,0,0,4,10,50);
        //Strobe(255,0x77,0,10,100,1000); //Slow
        //Strobe(255,255,255,10,50,1000); //Fast
        //Twinkle(255,0,0,10,100,false);
        //TwinkleRandom(20,100,false);
        //SnowSparkle(0x10, 0x10, 0x10, 20, getRandomNumber(100,1000));
        //RunningLights(255,255,0,50);  //Yellow
        //RunningLights(255,0,0,50); //RED
        //RunningLights(255,255,255,pixels,50);//WHITE
        //RunningLights(0,0,255,50); //BLUE
        colorWipe(0x00,0xff,0x00, pixels, 50, DMA_Buffer);
        colorWipe(0x00,0x00,0x00, pixels, 50, DMA_Buffer);
        //theaterChase(0xff,0,0,50);
        //Fire(55,120,15);
       //ChristmasLights();
       //theaterChaseRainbow(2); 
       //meteorRain(0xad,0x33,0xff,10, 64, true, 30);
    }
}
/*
 void ChristmasLightsStart(){
     // My Chrsitmas Light Display
        setAllRGB(0,0,0,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        
        setPixelRGB(25,0,0,0,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        
        setPixelRGB(25,0,0,1,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        setPixelRGB(25,0,0,2,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        setPixelRGB(25,0,0,3,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        setPixelRGB(25,0,0,4,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        setPixelRGB(25,0,0,5,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        setPixelRGB(25,0,0,6,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        
        setPixelRGB(50,0,0,4,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        
        setPixelRGB(50,0,0,2,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        
        setPixelRGB(50,0,0,3,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        
        setPixelRGB(128,0,0,2,pixels);
        setPixelRGB(128,0,0,3,pixels);
        setPixelRGB(128,0,0,4,pixels);
        writeLED(pixels,NUM_LEDS,DMA_Buffer);
        delay_ms(1000);
        uint8_t i;
        for (i=0; i < 10; i++){
            CylonBounce(255,0,0,4,10,50);
        }
}


void ChristmasLights(){
    uint8_t i;
    for (i=0; i < 6; i++){
        CylonBounce(255,255,255,4,10,50);
    }
    colorWipe(0x10,0x10,0x10,pixels,40);
    counter = 30000;
    while(counter){
        SnowSparkle(0x10, 0x10, 0x10, pixels, 20, getRandomNumber(100,1000));
    }
    
    Strobe(255,255,255,10,50,1000); //Fast
    
    colorWipe(0x32, 0x2d, 0x91,pixels,20);
    colorWipe(0xe6, 0xd1, 0x17,pixels,20);
    colorWipe(0x73, 0x12, 0x1f,pixels,20);
    colorWipe(0x61, 0xa3, 0xe6,pixels,20);
    colorWipe(0xe6, 0x89, 0x10,pixels,20);
    colorWipe(0x65, 0xbf, 0x74,pixels,20);
    
    CylonBounce(0x25,0x73,0x22,4,10,30);
    
    counter = 15000;
    while(counter){
        Sparkle(255,255,255,pixels,3); // White Sparkle
    }
    
    counter = 15000;
    while(counter){
        Sparkle(getRandomNumber(0,255),getRandomNumber(0,255),getRandomNumber(0,255),pixels,3);
    }
    

    RunningLights(255,255,255,pixels,30);//WHITE

    CylonBounce(0xad,0x1a,0x4b,4,10,30);

    for(i = 0; i<3; i++){
        theaterChase(0xff,0,0,30);
    }
    
    for(i = 0; i<3; i++){
        theaterChase(0x0c,0x5e,0x9c,20);
    }
    
     for(i = 0; i<3; i++){
        theaterChase(0x97,0x21,0x9C,10);
    }
    
    colorWipe(0xff, 0xbd, 0x38,pixels,20);
    colorWipe(0x17, 0xc5, 0x15,pixels,20);
    colorWipe(0x5c, 0x19, 0x1d,pixels,20);
    colorWipe(0x64, 0x37, 0x88,pixels,20);
    colorWipe(0xc4, 0x2d, 0xba,pixels,20);
    
    counter = 30000;
    while(counter){
        Fire(55,120,15);
    }
}
*/





/*

/*
void SnowSparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint16_t SparkleDelay, uint16_t SpeedDelay) {
    
     // SparkleDelay - Delay Time in ms (0-65535)
     // SpeedDelay   - Delay Time in ms (0-65535)
     // REQ: NUM_LEDS <= 255
     
    
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


*/
// TODO: ADD FadeInOut
// TODO: ADD rainbow cycle effect

// TODO@ ADD Bouncing Balls
// TODO: ADD Multi Colour Bouncing Balls

/*
void Fire(int Cooling, int Sparking, int SpeedDelay) {
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

void setPixelHeatColor (uint8_t Pixel, uint8_t temperature) {
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


void theaterChase(uint8_t R, uint8_t G, uint8_t B, uint8_t SpeedDelay) {
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

void theaterChaseRainbow(int SpeedDelay) {
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

uint8_t* Wheel(uint8_t WheelPos) {
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

void meteorRain(uint8_t R, uint8_t G, uint8_t B, uint8_t meteorSize, uint8_t meteorTrailDecay, bool meteorRandomDecay, int SpeedDelay) {  
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


void fadeToBlack(int ledNo, uint8_t fadeValue) {
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

void RunningLights(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3],  uint16_t WaveDelay) {
    
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

void Strobe(uint8_t R, uint8_t G, uint8_t B, uint8_t StrobeCount, uint16_t FlashDelay, uint16_t EndPause){

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

void Twinkle(uint8_t R, uint8_t G, uint8_t B, uint8_t Count, uint8_t SpeedDelay, bool OnlyOne) {
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

void TwinkleRandom(uint8_t Count, uint8_t SpeedDelay, bool OnlyOne) {
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
    










void DMA_Setup(){
    // DMA Setup - DMA1 Channel 5 for use with TIM2 Channel 1
    RCC->AHBENR |= 0x00000001; // Enable DMA 1 Clock
    
    // DMA Channel 5 Configuration
    // Load base address of peripheral register to which data will be written.
    // Load the address of the compare register converting from 16 to 32 bits
    DMA1_Channel5->CPAR = (uint32_t)(&(TIM2->CCR1));
    
    // Load the memory address data will be read from
    //DMA1_Channel5->CMAR = (uint32_t)pwm_array;
    DMA1_Channel5->CMAR = (uint32_t)DMA_Buffer;
    
    // Load the number of data transfers.
    // Circular mode will be used to loop over the two values in the array
    // so only to values need to be transferred before everyting resets.
    //DMA1_Channel5->CNDTR = 2;
    
    // Configure Channel Priority
    // Leaving at default 'Low' priority
    //DMA-CCR5 |= 0x00000000; 
    
    // Configure additional channel features
    //  Memory Size 8 bits
    //  Peripheral Size 16 bits
    //  Memory Increment Mode Enabled
    //  Circular Mode Enabled
    //  Direction: Read From Memory
    //  HT Interrupt: Enabled
    //  TC Interrupt: Enabled
    DMA1_Channel5->CCR |= 0x000001B6;
    
    NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn);    // Clear Pending Status
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);          // Enable Interrupt
        
    // Activate Channel
    //DMA1_Channel5->CCR |= 0x00000001;
    
}
void DMA1_Channel5_IRQHandler(void){

    uint8_t offset = 0;                     // DMA Buffer positon
    
    if (DMA1->ISR & 0x00040000){            // If Channel 5 HT Flag Set
        DMA1->IFCR = 0x00040000;            // Clear HT Flag
    }
    
    if (DMA1->ISR & 0x00020000){            // If Channel 5 TC Flag Set
        DMA1->IFCR = 0x00020000;            // Clear TC Flag
        offset = BYTES_PER_LED;             // Start of the 2nd half of buffer.        
    }
    
    // Load the next LED or Reset into buffer.
    if (currentLED < LED_COUNT){
        // Load the colour data into the DMA Buffer (at applied offset).
        loadColour(LEDSequence[currentLED], DMA_Buffer, offset);
    }else{
        loadReset(DMA_Buffer,offset);       // Load RESET Bytes
    }
   
    currentLED++;                           // Next LED
    
    // If two Reset cycles have been sent to finish the transfer sequence
    // disable the Timer and DMA.
    if (currentLED >= (LED_COUNT + 2)){
        // Disable Timer
        TIM2->CR1 &= ~0x0001;               // Clear Enable Bit
        // Disable DMA
        DMA1_Channel5->CCR &= ~0x00000001;   // Clear Enable Bit
    }
}
void PWM_Setup(){
    // Setup PWM using Timer2 (PA0, PA1)
    
    // Enable Clocks - Port A, Alternate Function
    RCC->APB2ENR |= 0x00000005;
    
    // Enable Timer 2
    RCC->APB1ENR |= 0x00000001;
    
    // Configure PA0, PA1 as AF Push-Pull Output Compare, 50MHz
    // Note: Reset Value For GPIOA-> is 0x44444444. Be careful when setting up
    // as incorrect function maybe selected.
    GPIOA->CRL = 0x000000BB;
    
    // Values Based on some example code
    // System Clock 72MHz
    // Timer Period (1/800kHz) = 0.00000125 seconds
    
    // SysClk/Timer Period = (Prescaler + 1)(Auto Reload Value +1)
    // 72x10^6/800x10^3 = 90
    
    // The values for the Prescaler and ARR are subject to interpretation on the 
    // ammount of steps (level of control) required over the output. The values
    // that follow are based on the example code of others and appear to have
    // been selected to make the maths easier.
    
    // Let Prescaler = 2
    
    // 90 = (Prescaler + 1)(Auto Reload Value +1)
    // 90 = 3(Auto Reload Value +1)
    // 90/3 = 30 = Auto Reload Value + 1
    // Auto Reload Value = 29
        
    // Set Timer 2 Reload Value
    TIM2->ARR = 0x001D; //29
    
    // Set Timer2 Prescaler Value
    TIM2->PSC = 0x0002; //2
    
    // Channel 1 & 2 - PWM Mode 2: 
    // Ch in upcounting is inactive as long as TIM2_CNT < TIM2_CCR2 else active.
    // Ch in downcounting is inactive as long as TIM2CNT>TIM2_CCR1 else active.
    TIM2->CCMR1 = 0x7878;
    // Alternatively
    // Channel 1 & 2 - PWM Mode 1: 
    // Ch in upcounting is active as long as 
    // TIM2_CNT<TIM2_CCR1
    // TIM2->CCMR1 = 0x6800;
    
    
    //Enable Update Generation
    TIM2->EGR |= 0x0001; // Reinitialise Counter & Update Registers
    
    // Channel 1 Enable
    TIM2->CCER |= 0x0003; // 0x0003 for active low, 0x0001 for active high
    
    // Channel 2 Enabled
    TIM2->CCER |= 0x0030; // 0x0030 for active low, 0x0010 for active high
    
    TIM2->CR2 |= 0x0004; // DMA Requests Sent when update event occurs.
    
    TIM2->DIER |= 0x0300; // Update DMA Request Enable, Ch 1 DMA Request Enable
    
    // Auto Preload Enable & Enable Timer Counter
    //TIM2->CR1 |= 0x0081;
    TIM2->CR1 |= 0x0080; // Preload Enabled, counter disabled
    
}
void PC13_LED_Setup(){
    // PortC GPIO Setup
    // Enable I/O Clock for PortC
    RCC->APB2ENR |= 0x00000010;

    // Configure PC13
    // - LED Indicator
    // - General Purpose Output Push-Pull
    // - Max Speed 50MHz
    
    GPIOC->CRH &= 0xFF0FFFFF; // Zero Settings for PC13, preserve the rest
    GPIOC->CRH |= 0x00300000; // Apply Config to PC13 (50MHz)
}

void toggleLed(){
    // Toggle the LED attached to PC13
    if (flash == 0){
        // Get LED Status
        if (GPIOC->ODR & 0x00002000){ // If Set
            GPIOC->BSRR |= 0x20000000; // Reset
        }else{                        // If Not Set
            GPIOC->BSRR |= 0x00002000; // Set
        }
        flash = 1001;
    }
}



void SysTick_Handler(void){
    if (ticks != 0){
        // Pre-decrement ticks. This avoids making a copy of the variable to 
        // decrement. This should be faster which is ideal for an interrupt
        // service routine.
        // Note: Since the value will pre-decrement the value of ticks will need
        // to be incremented by 1 to be correct.
        --ticks;
    }
    if (flash !=0){ // Decrement the LED Toggle Counter
        --flash;
    }
    
    if (counter !=0){ // Decrement the counter
        --counter;
    }

}

