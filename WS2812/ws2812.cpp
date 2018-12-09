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
#include "delay.h"              // Simple Delay Function
#include "stm32f103xb.h"        // HW Specific Header


// LED Definitions
#define BYTES_PER_LED 24 // Number of bytes holding colour data for each LED.
#define NUM_LEDS 60
////////////////////////////////////////////////////////////////////////////////
// Function Declarations
extern "C" void SysTick_Handler(void);
//extern "C" void TIM2_IRQHandler(void);
extern "C" void DMA1_Channel5_IRQHandler(void);
void SysTick_Init(void);
void delay_ms(uint32_t ms);
void toggleLed();
void changeColour();
void PWM_Setup();      // Timer 2 PWM - Ch1 & Ch2
void DMA_Setup();
void Timebase_Setup(); // Timebase from Timer using interrupts
void PC13_LED_Setup(); // Setup PC13 for output LED
void writeLED(uint8_t (*colour)[3], uint8_t length, uint8_t *buffer);
void loadReset(uint8_t *array, uint8_t offset);
void setPixel(uint8_t colour[3], uint8_t pixel, uint8_t (&array)[NUM_LEDS][3]);
void setPixelRGB(uint8_t R, uint8_t G, uint8_t B, uint8_t pixel, uint8_t (&array)[NUM_LEDS][3]);
void setAll(uint8_t colour[3], uint8_t (&array)[NUM_LEDS][3]);
void setAllRGB(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3]);
int getRandomNumber(int min, int max);
// Effects
void RGBLoop(uint8_t (&array)[NUM_LEDS][3]);
void Sparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint8_t SpeedDelay);
void RunningLights(uint8_t R, uint8_t G, uint8_t B,  uint8_t WaveDelay);
void SnowSparkle(uint8_t R, uint8_t G, uint8_t B,  uint8_t SparkleDelay, uint8_t SpeedDelay);
void CylonBounce(uint8_t R, uint8_t G, uint8_t B, int EyeSize, int SpeedDelay, int ReturnDelay);
void Strobe(uint8_t R, uint8_t G, uint8_t B, uint8_t StrobeCount, uint8_t FlashDelay, uint8_t EndPause);
void Twinkle(uint8_t R, uint8_t G, uint8_t B, uint8_t Count, uint8_t SpeedDelay, bool OnlyOne);
void TwinkleRandom(uint8_t Count, uint8_t SpeedDelay, bool OnlyOne);
void colorWipe(uint8_t R, uint8_t G, uint8_t B, uint8_t SpeedDelay);
void theaterChase(uint8_t R, uint8_t G, uint8_t B, uint8_t SpeedDelay);

void setPixelHeatColor (uint8_t Pixel, uint8_t temperature);
void Fire(int Cooling, int Sparking, int SpeedDelay);

////////////////////////////////////////////////////////////////////////////////
// Buffers
// -------
// Create buffers - Size defined by buffer_class.h or variable for compiler.
//Buffer serial_rx;
//Buffer serial_tx;
//Buffer rx_buffer;
////////////////////////////////////////////////////////////////////////////////
// Defines
// -------


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

static uint8_t colour0[][3] = {RED, GREEN, OFF, WHITE, BLUE}; // Length 5
static uint8_t colour1[][3] = {RED, RED, WHITE, BLUE, BLUE}; // Length 5
static uint8_t colour2[][3] = {BRIGHTWHITE, BRIGHTWHITE, BRIGHTWHITE, BRIGHTWHITE,BRIGHTWHITE}; // Length 5

static uint8_t colour3[][3] = {GREEN, GREEN, GREEN, GREEN, GREEN};
static uint8_t colour4[][3] = {RED, GREEN, GREEN, GREEN, GREEN};
static uint8_t colour5[][3] = {GREEN, RED, GREEN, GREEN, GREEN};
static uint8_t colour6[][3] = {GREEN, GREEN, RED, GREEN, GREEN};
static uint8_t colour7[][3] = {GREEN, GREEN, GREEN, RED, GREEN};
static uint8_t colour8[][3] = {GREEN, GREEN, GREEN, GREEN, RED};
static uint8_t colour9[][3] = {PINK, PINK, PINK, PINK, PINK};
////////////////////////////////////////////////////////////////////////////////
// Global Variables
volatile uint32_t ticks = 0;        // Used for SysTick count down.
volatile uint32_t flash = 0;        // Used for PC13 LED Flash Toggle Interval
volatile uint32_t colour_change = 6001; // Used LED Colour Change Interval

uint8_t colour_rotation = 0;         // Used in loading colour sequence.

static uint8_t currentLED = 0;      // Tracks LED write progress
// Points to the colour sequence being sent. Used to allow DMA_ISR to load
// data into buffer.
static uint8_t (*LEDSequence)[3];

// The DMA Buffer needs to be able to hold the data for 2 LEDs. When the data
// for one LED is sent the DMA HT (Half Transfer) Flag is set. After the data 
// for the next LED is send the DMA TC (Transfer Complete) Flag is set. 
// The DMA transfer is setup for cicular mode and will then wrap around to the
// start.
uint8_t DMA_Buffer[2*BYTES_PER_LED] = {};
////////////////////////////////////////////////////////////////////////////////
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
        
        /*
        setAllRGB(0,0,0,pixels);
        
        setPixel(RED2,0,pixels);
        setPixelRGB(0,0,63,1,pixels);
        setPixelRGB(255,20,147,2,pixels);
        writeLED(pixels,NUM_LEDS, DMA_Buffer);
        delay_ms(1000);
        
        setAllRGB(138,43,226, pixels);
        writeLED(pixels,NUM_LEDS, DMA_Buffer);
        delay_ms(1000);
        
        setAllRGB(255,255,255, pixels);
        writeLED(pixels,NUM_LEDS, DMA_Buffer);
        delay_ms(1000);
        
        setAllRGB(250,20,147, pixels);
        writeLED(pixels,NUM_LEDS, DMA_Buffer);
        delay_ms(4000);
        
        setAll(RED2, pixels);
        writeLED(pixels,NUM_LEDS, DMA_Buffer);
        delay_ms(1000);
        */
        //RGBLoop(pixels);
        
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
        //RunningLights(255,255,255,50);//WHITE
        //RunningLights(0,0,255,50); //BLUE
        //colorWipe(0x00,0xff,0x00, 50);
        //colorWipe(0x00,0x00,0x00, 50);
        //theaterChase(0xff,0,0,50);
        Fire(55,120,15);
  
        
    }
}


int getRandomNumber(int min, int max){
    // Generate a random number between min and max (inclusive)
    // Assumes std::srand() has already been called
    // Assumes max - min <= RAND_MAX
    static const double fraction = 1.0 / (RAND_MAX + 1.0);  // static used for efficiency, so we only calculate this value once
    // evenly distribute the random number across our range
    return min + static_cast<int>((max - min + 1) * (std::rand() * fraction));
}


void CylonBounce(uint8_t R, uint8_t G, uint8_t B, int EyeSize, int SpeedDelay, int ReturnDelay){
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

void RGBLoop(uint8_t (&array)[NUM_LEDS][3]){
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


void Sparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint8_t SpeedDelay) {

  uint8_t Pixel = getRandomNumber(0,NUM_LEDS);
  setPixelRGB(R,G,B,Pixel,array);
  writeLED(array,NUM_LEDS,DMA_Buffer);
  delay_ms(SpeedDelay);
  setPixelRGB(0,0,0,Pixel,array);
}

void SnowSparkle(uint8_t R, uint8_t G, uint8_t B,  uint8_t SparkleDelay, uint8_t SpeedDelay) {
  setAllRGB(R,G,B,pixels);
  uint8_t Pixel = getRandomNumber(0,NUM_LEDS);
  setPixelRGB(0xff,0xff,0xff, Pixel,pixels);
  writeLED(pixels,NUM_LEDS,DMA_Buffer);
  delay_ms(SparkleDelay);
  setPixelRGB(R,G,B,Pixel,pixels);
  writeLED(pixels,NUM_LEDS,DMA_Buffer);
  delay_ms(SpeedDelay);
}

void colorWipe(uint8_t R, uint8_t G, uint8_t B, uint8_t SpeedDelay) {
  for(uint8_t i=0; i<NUM_LEDS; i++) {
      setPixelRGB(R, G, B, i, pixels);
      writeLED(pixels,NUM_LEDS,DMA_Buffer);
      delay_ms(SpeedDelay);
  }
}
// TODO: ADD FadeInOut
// TODO: ADD rainbow cycle effect
// TODO: ADD Theatre Chase Rainbow Effect
// TODO@ ADD Bouncing Balls
// TODO: ADD Multi Colour Bouncing Balls
// TODO: ADD Meteor Rain Effect

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


void RunningLights(uint8_t R, uint8_t G, uint8_t B,  uint8_t WaveDelay) {
  uint8_t Position=0;
  for(uint8_t j=0; j<NUM_LEDS*2; j++){
      Position++; // = 0; //Position + Rate;
      for(uint8_t i=0; i<NUM_LEDS; i++) {
        // sine wave, 3 offset waves make a rainbow!
        //float level = sin(i+Position) * 127 + 128;
        //setPixelRGB(level,0,0,i, pixels);
        //float level = sin(i+Position) * 127 + 128;
        setPixelRGB(((sin(i+Position) * 127 + 128)/255)*R,
                   ((sin(i+Position) * 127 + 128)/255)*G,
                   ((sin(i+Position) * 127 + 128)/255)*B,
                   i,pixels);
      }
      writeLED(pixels,NUM_LEDS,DMA_Buffer);
      delay_ms(WaveDelay);
  }
}

void Strobe(uint8_t R, uint8_t G, uint8_t B, uint8_t StrobeCount, uint8_t FlashDelay, uint8_t EndPause){

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

void setPixel(uint8_t colour[3], uint8_t pixel, uint8_t (&array)[NUM_LEDS][3]){
    // pixel is the LED position in the string
    // colour is an array of R,G,B values
    // array is the array which holds the data for all the pixels in the string.
    array[pixel][0] = colour[0];
    array[pixel][1] = colour[1];
    array[pixel][2] = colour[2];
    }

void setPixelRGB(uint8_t R, uint8_t G, uint8_t B, uint8_t pixel, uint8_t (&array)[NUM_LEDS][3]){
    // pixel is the LED position in the string
    // R,G,B are individual colour values.
    // array is the array which holds the data for all the pixels in the string.
    array[pixel][0] = R;
    array[pixel][1] = G;
    array[pixel][2] = B;
    }

void setAll(uint8_t colour[3], uint8_t (&array)[NUM_LEDS][3]){
    // R,G,B are individual colour values.
    // array is the array which holds the data for all the pixels in the string.
    for (uint8_t i = 0; i < NUM_LEDS; i++){
            array[i][0] = colour[0];
            array[i][1] = colour[1];
            array[i][2] = colour[2];   
    }
}

void setAllRGB(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3]){
    // R,G,B are individual colour values.
    // array is the array which holds the data for all the pixels in the string.
    for (uint8_t i = 0; i < NUM_LEDS; i++){
            array[i][0] = R;
            array[i][1] = G;
            array[i][2] = B;   
    }
}


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
/*
void changeColour(){
    if (colour_change == 0){
        // Change the colours of the LEDS
        // Note: Currently the Timer & DMA are stopped after the previous
        //       transfer. Allowing time to load the data into the array. 
        //       Need to restart them to allow the colour change to complete.
        if (colour_rotation == 0){
            loadColour(RED,pwm_array,0);
            loadColour(GREEN,pwm_array,24);
        } 
        else if (colour_rotation == 1){
            loadColour(GREEN,pwm_array,0);
            loadColour(BLUE,pwm_array,24);
        }
        else if (colour_rotation == 2){
            loadColour(BLUE,pwm_array,0);
            loadColour(WHITE,pwm_array,24);
        }
        else{
            loadColour(OFF,pwm_array,0);
            loadColour(OFF,pwm_array,24);
        }
        
        // Select next colour rotation, loop around at end.
        if (colour_rotation > 2){
            colour_rotation = 0; // Reset Colour Rotation
        }
        else{
            colour_rotation++; 
        }
        
        //DMA1_Channel5->CNDTR = 96;
        // Enable DMA (Before Timer)
        DMA1_Channel5->CCR |= 0x00000001;
        // Enable Timer
        TIM2->CR1 |= 0x0001;
        
        
        // Reset the Counter
        colour_change = 6001; // 6 Seconds based on 1ms SysTick
    }
}
*/
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

void SysTick_Init(void){
    // SystemCoreClock/1000     =  1ms
    // SystemCoreClock/100000   = 10us
    // SystemCoreClock/1000000  =  1us
    
    // Triggering the interupt every 1us is probably excessive unless there is a
    // specific reason. An interrupt every 1ms should be sufficient.
    
    // SysTick_Config() is defined in core_cm3.h
    // Value of SystemCoreClock is defined in startup file.
    
    // Setup and start SysTick
    while(SysTick_Config(SystemCoreClock/1000) != 0){
        // One SysTick Should now equal 1ms
        // This means the interrupt SysTick_Handler() will run every 1ms. 
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
    
    //if (colour_change !=0){ // Decrement the Colour Change Counter
    //    --colour_change;
    //}
}

void delay_ms(uint32_t ms){
    
    if (ms != 0xFFFFFFFF){
        // Increment the delay count by one to compensate for the pre-decrement
        // in the SysTick handler.
        // The only instance where this will not happen is when the delay has
        // been set to its maximum value. A 1ms difference over the maximum 
        // delay is insignificant and is ignored. A 1ms difference will be more 
        // apparent over shorter delays hence the need for compensation.
        ++ms;
    }
    ticks = ms;
    while(ticks);
    
}
