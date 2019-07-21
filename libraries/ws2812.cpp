// ws2812.cpp
#include "ws2812.h"
////////////////////////////////////////////////////////////////////////////////

// Used by DMA1_Channel5_IRQHandler & writeLED
uint8_t currentLED = 0;      // Tracks LED write progress

// Points to the colour sequence being sent. Used to allow DMA_ISR to load
// data into buffer.
uint8_t (*LEDSequence)[3];

uint8_t LED_COUNT = NUM_LEDS;      // Number of LEDs in string.

// The DMA Buffer needs to be able to hold the data for 2 LEDs. When the data
// for one LED is sent the DMA HT (Half Transfer) Flag is set. After the data 
// for the next LED is send the DMA TC (Transfer Complete) Flag is set. 
// The DMA transfer is setup for cicular mode and will then wrap around to the
// start.

uint8_t DMA_Buffer[2*BYTES_PER_LED] = {};
uint8_t pixels[NUM_LEDS][3]= {0};    // Array of LED Data

// WS2812 Functions
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
void PWM_Setup(){
    // Setup PWM using Timer2 (PA0, PA1)
    
    // Enable Clocks - Port A, Alternate Function
    RCC->APB2ENR |= 0x00000005;
    
    // Enable Timer 2
    RCC->APB1ENR |= 0x00000001;
    
    // Configure PA0, PA1 as AF Push-Pull Output Compare, 50MHz                 //<--- Do I need to set PA1 since only PA0 is sending data out??
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
void DMA1_Channel5_IRQHandler(){

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

void setAllRGB(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3]){
    // R,G,B are individual colour values.
    // array is the array which holds the data for all the pixels in the string.
    for (uint8_t i = 0; i < NUM_LEDS; i++){
            array[i][0] = R;
            array[i][1] = G;
            array[i][2] = B;   
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
