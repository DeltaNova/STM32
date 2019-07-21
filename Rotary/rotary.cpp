// rotary.cpp
// Program to use a rotart encoder
//  
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>             // Enable fixed width integers
#include <stdio.h>              // Newlib-nano
#include "clock.h"              // Setup system and peripheral clocks
#include "count.h"              // Count Related Functions
#include "buffer_class.h"       // Cicular Buffers
#include "button.h"             // Button Library
#include "serial.h"             // USART1 Setup & Functions
#include "stm32f103xb.h"        // HW Specific Header
#include "systick.h"            // SysTick Configuration
extern volatile uint32_t ticks; // SysTick Library
////////////////////////////////////////////////////////////////////////////////
// Global Variables
volatile uint32_t flash = 0;        // Used for PC13 LED Flash Toggle Interval
volatile uint32_t counter = 0;      // Systick incremented counter

////////////////////////////////////////////////////////////////////////////////
// WS2812 Variables
// Used by DMA1_Channel5_IRQHandler & writeLED
static uint8_t currentLED = 0;      // Tracks LED write progress
// Points to the colour sequence being sent. Used to allow DMA_ISR to load
// data into buffer.
static uint8_t (*LEDSequence)[3];

#define NUM_LEDS 5

uint8_t LED_COUNT = NUM_LEDS;      // Number of LEDs in string.

// The DMA Buffer needs to be able to hold the data for 2 LEDs. When the data
// for one LED is sent the DMA HT (Half Transfer) Flag is set. After the data 
// for the next LED is send the DMA TC (Transfer Complete) Flag is set. 
// The DMA transfer is setup for cicular mode and will then wrap around to the
// start.
#define BYTES_PER_LED 24
uint8_t DMA_Buffer[2*BYTES_PER_LED] = {};
static uint8_t pixels[NUM_LEDS][3]= {0};    // Array of LED Data
extern "C" void DMA1_Channel5_IRQHandler(void);
////////////////////////////////////////////////////////////////////////////////
// WS2812 Function Declarations
void DMA_Setup();
void DMA1_Channel5_IRQHandler();
void PWM_Setup();
void loadColour(uint8_t *colour, uint8_t *array, uint8_t offset);
void loadReset(uint8_t *array, uint8_t offset);
void writeLED(uint8_t (*colour)[3], uint8_t length, uint8_t *buffer);
////////////////////////////////////////////////////////////////////////////////
// Function Declarations
extern "C" void USART1_IRQHandler(void);
extern "C" void SysTick_Handler(void);
void toggleLed();
void PC13_LED_Setup(); // Setup PC13 for output LED
void EncoderSetup();
void EncoderButtonSetup();
void update_encoder_counts();

// Returns the current counter value of the SysTick incremented count.
uint32_t getSysTickCount();         

// Struct to hold a value with range limits
struct Value {
    uint8_t value = 0;
    uint8_t valueMin = 0;
    uint8_t valueMax = 255;
};
void updateValue(Value &value, uint16_t dir, uint16_t delta);
void ValueShow(Value& value, Serial& serial, char *char_buffer);
void ValueShowMenu(Value& value, Serial& serial, char *char_buffer);

void processMenuSelection(Serial& serial, char *char_buffer,Value &MenuSelection, 
    Value &Red, Value &Green, Value &Blue);
                          
// USART1
Buffer serial_tx; // USART1 TX Buffer (16 bytes)
Buffer serial_rx; // USART1 RX Buffer (16 bytes)

// Rotary Encoder
static uint16_t encoder_count = 0;
static uint16_t last_encoder_count = 0; 

// These button related functions are application specific and dont belong
// in the button library.
void processButtonAction(pressType ButtonAction, Serial& serial, char *char_buffer, 
    Value &MenuSelection, Value &Red, Value &Green, Value &Blue);                     
void pressShort(Serial& serial, char *char_buffer, Value &MenuSelection, 
    Value &Red, Value &Green, Value &Blue);
void pressLong(Serial& serial, char *char_buffer,  Value &MenuSelection, 
    Value &Red, Value &Green, Value &Blue);
void pressVlong(Serial& serial, char *char_buffer, Value &MenuSelection, 
    Value &Red, Value &Green, Value &Blue);
void showMenu(Serial& serial, char *char_buffer, Value &MenuSelection, 
    Value &Red, Value &Green, Value &Blue);

// Timeout Reset Value (ms)for Idle state
#define TIMEOUT 5000
static uint16_t idle = TIMEOUT;
// History of button state.
static uint32_t button_history = 0; 

// Strings & Initial Values (Escape Chars 1 byte each_
static uint8_t test_message[] = "Waiting!\n\r";         //Size 10
static uint8_t idle_message[] = "Idle\n\r";             // Size 6

static uint8_t Menu_Header[] = "Menu";      // Size 4
static uint8_t Menu0[] = "0: Exit";         // Size 7
static uint8_t Menu1[] = "1: Red   ";       // Size 9
static uint8_t Menu2[] = "2: Green ";       // Size 9
static uint8_t Menu3[] = "3: Blue  ";       // Size 9
static uint8_t Menu4[] = "4: Set";          // Size 6

enum class State{
    IDLE, 
    MENU, 
    RED, 
    GREEN, 
    BLUE,
};
    State state = State::IDLE;  // Holds current program state
////////////////////////////////////////////////////////////////////////////////
// Main - Called by the startup code.
int main() {
    ClockSetup();       // Setup System & Peripheral Clocks
    SysTick_Init();     // Enable SysTick
    
    // Create instance of Serial class (USART1) 
    Serial serial(serial_rx, serial_tx);     
    serial.setup();     // Enable Serial Support - Currently USART1 Specific
    // NOTE: Tie USART RX Pin low as I suspect interrupt causing problems.

    PC13_LED_Setup();   // Setup PC13 for output LED
    EncoderSetup();     // Setup Rotary Encoder
    EncoderButtonSetup(); // Setup the Rotary Encoder Button
    
    ////////////////////////////////////////////////////////////////////////////
    // WS2812 HW Setup
    DMA_Setup();
    PWM_Setup();

    // The counter will count to the reload value where upon it will toggle the
    // output state of each channel back to its reset value. 
    // During the count, before the reload value is reached, the count is 
    // compared with each channels compare value. If the compare value is
    // matched the output state of the matched channel will be toggled.
    
    // Timer 2 Channel 1 Compare Value
    TIM2->CCR1 = 0x0000;        // 0 (logic 1)
    // Timer 2 Channel 2 Compare Value
    TIM2->CCR2 = 0x0009;        // 9 (Logic 0)
        
    
    ////////////////////////////////////////////////////////////////////////////
    // NOTE: char_buffer needs to be sized to be able to hold longest message.
    char char_buffer[16];
    // Send a message to the terminal to indicate program is running.
    serial.write_array(test_message,10);
    serial.write_buffer();
    
    
    Value V;                // Used to simply output of selected value.
    Value NullValue;        // Create instance of Value Struct
    Value MenuSelection;    // Holds the current and range of menu selections.
    MenuSelection.valueMax = 4; 
    // Menu Options:
    // 0 Exit
    // 1 Red
    // 2 Green
    // 3 Blue
    // 4 Set
    
    // Values of RGB colour settings
    Value Red;
    Value Green;
    Value Blue;
    
    Button B;               // Create an instance of the encoder button.

    
    while(1){
                        // Triggers Every Second
        toggleLed();    // Toggle LED (PC13) to indicate loop operational
        
        
        if (idle == 0x01){ // About to enter IDLE state
            // Print Idle Message Here
            serial.lineClear(6);
            serial.write_array(idle_message,6);                                 
            serial.write_buffer();
            // Count decremented to zero by the next Systick. 
            // Generating the message before getting to zero to prevent constant 
            // retriggering whilst in the idle state.
            state = State::IDLE;
        }
               
        // Assess what the button is doing and trigger appropritate action.
        pressType p = buttonAction(&button_history,B,getSysTickCount());
        processButtonAction(p, serial, char_buffer, MenuSelection, Red, Green, Blue);
        
        
        // Update Encoder Counts then check for any movement of the encoder.
        update_encoder_counts();
        if ((encoder_count/4) != (last_encoder_count/4)) {  
            // encoder_count & last_encoder_count values are divided by 4 
            // before use. This is to reflect the 4 clock pulses per detent.
            // The result is the following code executes every detent.
            
            idle = TIMEOUT; // Reset idle timeout due to user interraction
            
            // Direction Encoder Moved
            uint16_t dir = (TIM3->CR1 & 0x0010); 
            // Ammount Encoder Moved
            uint16_t delta = getCountDiff(encoder_count,last_encoder_count);
            
            // Select the value to be adjusted based on the program state.
            switch(state){
                case State::MENU:
                    updateValue(MenuSelection,dir, delta);
                    V = MenuSelection;
                    break;
                    
                case State::RED:
                    updateValue(Red,dir,delta);
                    V = Red;
                    break;
                    
                case State::GREEN:
                    updateValue(Green,dir,delta);
                    V = Green;
                    break;
                    
                case State::BLUE:
                    updateValue(Blue,dir,delta);
                    V = Blue;
                    break;
                    
                default:
                    updateValue(NullValue,dir, delta);
                    V = NullValue;
                    break;
            }

            // Only show value for specific states, fall through list.
            // Should avoid the output of NullValue
            switch(state){ 
                case State::MENU:
                case State::RED:
                case State::GREEN:
                case State::BLUE:
                    // Output Backspaces to clear previous value before overwriting.
                    for (uint8_t i=0;i<3; i++){
                        serial.write(0x08);
                    }  

                    // Output Updated Value
                    ValueShowMenu(V, serial, char_buffer);
                    break;
                default:
                    break;
            }

        }else{  // No change in encoder count
            if (is_button_down(&button_history)){
                // Button Down - User interraction
                idle = TIMEOUT;             
            }   // Else No user Interraction
        }
    }
}
void showMenu(Serial& serial, char *char_buffer, Value &MenuSelection, 
    Value &Red, Value &Green, Value &Blue){
    // Display Menu with values for the colour variables.
    serial.newline();
    serial.write_array(Menu_Header,4);
    serial.write_buffer();
    serial.newline();
    
    serial.write_array(Menu0,7);
    serial.write_buffer();
    serial.newline();
    
    serial.write_array(Menu1,9);
    serial.write_buffer();
    ValueShowMenu(Red,serial,char_buffer);    
    serial.newline();
    
    serial.write_array(Menu2,9);
    serial.write_buffer();
    ValueShowMenu(Green,serial,char_buffer);  
    serial.newline();
    
    serial.write_array(Menu3,9);
    serial.write_buffer();
    ValueShowMenu(Blue,serial,char_buffer);  
    serial.newline();
    
    serial.write_array(Menu4,6);
    serial.write_buffer();
    serial.newline();
    
    // Leave a blank like between the  Menu and the Menu Selection Line
    serial.newline();       
    
    // Add Menu Line Prefix
    serial.write(0x4D);     // M
    serial.write(0x3A);     // :
    serial.write(0x20);     // SPACE
    
    // Reset the menuselection to zero each time menu is shown.
    MenuSelection.value = MenuSelection.valueMin;      
    ValueShowMenu(MenuSelection, serial, char_buffer);
    
}

void processButtonAction(pressType ButtonAction, Serial& serial, char *char_buffer, 
    Value &MenuSelection, Value &Red, Value &Green, Value &Blue){
    // Execute an action based on the button press duration.
    // ButtonAction is the only required parameter for this function.
    // Additional paramters are for passing to action functions.
    switch(ButtonAction){
        case pressType::SHORT:
            pressShort(serial, char_buffer, MenuSelection, Red, Green, Blue);
            break;
        case pressType::LONG:
            pressLong(serial, char_buffer, MenuSelection, Red, Green, Blue);
            break;
        case pressType::VLONG:
            pressVlong(serial, char_buffer, MenuSelection, Red, Green, Blue);
            break;
        case pressType::IDLE:
        default:
            break;
            
    }
}
void pressShort(Serial& serial, char *char_buffer, Value &MenuSelection, 
    Value &Red, Value &Green, Value &Blue){
    // Short Button Press Event
    switch(state){
        case State::IDLE:
            showMenu(serial,char_buffer,MenuSelection, Red,Green,Blue);
            state = State::MENU;
            break;
        case State::MENU:
            // Button Press to Process Menu Selection
            processMenuSelection(serial, char_buffer, MenuSelection, Red, Green, Blue);
            break;
        case State::RED:
            serial.lineClear(6);
            showMenu(serial,char_buffer,MenuSelection, Red,Green,Blue);
            state = State::MENU;
            break;
        case State::GREEN:
            serial.lineClear(6);
            showMenu(serial,char_buffer,MenuSelection, Red,Green,Blue);
            state = State::MENU;
            break;
        case State::BLUE:
            serial.lineClear(6);
            showMenu(serial,char_buffer,MenuSelection, Red,Green,Blue);
            state = State::MENU;
            break;
        default:
            break;
    }
    
}
void processMenuSelection(Serial& serial, char *char_buffer, Value &MenuSelection, 
    Value &Red, Value &Green, Value &Blue){
    // Change program flow based on selected menu option.
    serial.lineClear(3);
    switch(MenuSelection.value){
        case 0:     // Exit
            idle = 0x01;        // Force Idle State
            state = State::IDLE;
            break;
        case 1:     // Red
            state = State::RED;
            serial.write(0x52);  // R
            serial.write(0x3A);  // :
            serial.write(0x20);  // SPACE
            ValueShowMenu(Red,serial,char_buffer);
            //serial.newline();
            break;
        case 2:     // Green
            state = State::GREEN;
            serial.write(0x47);     // G
            serial.write(0x3A);     // :
            serial.write(0x20);     // SPACE
            ValueShowMenu(Green,serial,char_buffer);
            //serial.newline();
            break;
        case 3:     // Blue
            state = State::BLUE;
            serial.write(0x42);     // B
            serial.write(0x3A);     // :
            serial.write(0x20);     // SPACE
            ValueShowMenu(Blue,serial,char_buffer);
            //serial.newline();
            break;
        case 4:     // Set                            // TODO: Add in Set Action
            state = State::IDLE; // Return to IDLE after Set Action
            serial.write(0x49);  // I
            serial.newline();
            break;
        default:    // Default to Exit
            state = State::IDLE;
            serial.write(0x49);  // I
            serial.newline();
            break;
    }
}
void pressLong(Serial& serial, char *char_buffer, Value &MenuSelection, 
    Value &Red, Value &Green, Value &Blue){
    // Long Button Press Event
    switch(state){
        case State::IDLE:
            // No alternative action, default to short press
            pressShort(serial, char_buffer,MenuSelection, Red, Green, Blue);
            break;
        case State::MENU:
            // No alternative action, default to short press
            pressShort(serial, char_buffer,MenuSelection, Red, Green, Blue);
            break;
        case State::RED:
            // Zero Red Value, but remain in R
            Red.value = Red.valueMin; 
            
            // Output Backspaces to clear previous value before overwriting.
            for (uint8_t i=0;i<3; i++){
                serial.write(0x08);
            }
            
            // Show Reset Value  
            ValueShowMenu(Red, serial, char_buffer);
            
            state = State::RED;         // Retain Current State
            break;
        case State::GREEN:
            // Zero Green Value, but remain in G
            Green.value = Green.valueMin; 

            // Output Backspaces to clear previous value before overwriting.
            for (uint8_t i=0;i<3; i++){
                serial.write(0x08);
            }
            
            // Show Reset Value  
            ValueShowMenu(Green, serial, char_buffer);

            state = State::GREEN;       // Retain Current State
            break;
        case State::BLUE:
            // Zero Blue Value, but remain in B
            Blue.value = Blue.valueMin; 
            
            // Output Backspaces to clear previous value before overwriting.
            for (uint8_t i=0;i<3; i++){
                serial.write(0x08);
            }
            
            // Show Reset Value  
            ValueShowMenu(Blue, serial, char_buffer);
            
            state = State::BLUE;        // Retain Current State
            break;
        default:
            pressShort(serial, char_buffer,MenuSelection, Red, Green, Blue);
            break;
    }
    
}
void pressVlong(Serial& serial, char *char_buffer, Value &MenuSelection, 
    Value &Red, Value &Green, Value &Blue){
    // Very Long Button Press Event
    // Zero All Colour Values and return to Menu state.
    Red.value = Red.valueMin;
    Green.value = Green.valueMin;
    Blue.value = Blue.valueMin;
    showMenu(serial,char_buffer,MenuSelection, Red,Green,Blue);
    state = State::MENU;
    
 
    
    //TODO: I plan to use a SET state to turn LED's on, UNSET here to turn off.
}

void updateValue(Value &value, uint16_t dir, uint16_t delta){
    // Apply the delta to current value.
    uint16_t i = 0;
    if (dir){   // If TRUE then count DOWN
        for (i=0; i < delta; i++){
            if (value.value > value.valueMin){
                value.value--;
            }
        }
    }else{      // If FALSE then count UP
        for (i=0; i < delta; i++){
            if (value.value < value.valueMax){
                value.value++;    
            }
        }
    }
}
void ValueShow(Value& value, Serial& serial, char *char_buffer){
    // Output Value (5 Digits)
    snprintf(char_buffer, 8, "%05u", value.value);
    for(uint8_t i=0;i<5; i++){
        serial.write(char_buffer[i]);
    }
}
void ValueShowMenu(Value& value, Serial& serial, char *char_buffer){
    // Output Value (3 Digits for Menu)
    snprintf(char_buffer, 8, "%03u", value.value);
    for(uint8_t i=0;i<3; i++){
        serial.write(char_buffer[i]);
    }
}

void EncoderButtonSetup(){
    // Button will be using PB6 and is wired in with a pullup resistor
    RCC->APB2ENR |= 0x00000009;     // Enable Clocks (AFIO, Port B)
    // Clear PB6 Bits Before Setting as register does not zero on reset.
    GPIOB->CRL &= 0xF0FFFFFF;
    GPIOB->CRL |= 0x08000000;       // Setup PB6 as an input
    GPIOB->ODR |= 0x00000040;       // Set PB6 to use Pullup
}
void EncoderSetup(){
    // Setup For Rotary Encoder
    // Using Timer3 with partial remap for use of PB4 & PB5
    RCC->APB2ENR |= 0x00000009;     // Enable Clocks (AFIO, Port B)
    AFIO->MAPR |= 0x00000800;       // Partial Remap of Timer 3 Ports
    RCC->APB1ENR |= 0x00000002;     // Enable Clocks (TIM3)
    
    // Configure PB4 & PB5 (Floating Input, Input Mode)
    GPIOB->CRL &= 0xFF00FFFF;       // Clear PB4/PB5 Bits
    GPIOB->CRL |= 0x00440000;       // Set ports as inputs   
        
    // Setup Timer 3
    // CKD = 00 (Div/1)
    // ARPE = 0 (Not Bufffered)
    // CMS = 00 (Edge Alighed)
    // DIR = 0 (Up Counter)
    // OPM = 0 (Counter Not Stopped at Update Event)
    // URS = 0 (Update Request Sources)
    // UEV = 0 (Update Events Enabled)
    // CEM = 0 (Counter Disabled) <- Gets Enabled Later
    TIM3->CR1 = 0x0000;
    
    TIM3->SMCR |= 0x0003;           // Set SMS Bits for Encoder Mode 3
    
    // Ensure CC1E = 0 , CC2E = 0 to allow setting of capture/compare bits.
    TIM3->CCER &= 0xFFEE;
        
    // Set Capture/Compare Mode
    // CC1S = 01, CC2S = 01 (IC1 is mapped on TI1, IC2 is mapped on TI2)
    // IC1F = 0000, IC2F = 0000 (No filtering)
    // IC1PSC = 10, IC2PSC = 10 (Capture Every 4 Events)
    TIM3->CCMR1 |= 0x0909;
    
    // Set Polarity and Enable Capture/Compare
    // CC1P = 0, CC2P = 1
    // CC1E = 1, CC2E = 1
    TIM3->CCER |= 0x0031;
    
    // Set Max Count Value (Count up between 0 and ARR, count down ARR to 0)
    TIM3->ARR = 0xFFFF;             // 65535
    
    //Enable Update Generation
    TIM3->EGR |= 0x0001;            // Reinitialise Counter & Update Registers
    TIM3->CR1 |= 0x00001;           // Enable Timer 3
}

void update_encoder_counts(){
    // Read the hardware counter connected to the rotary encoder.
    last_encoder_count = encoder_count;     // Store previous encoder_count 
    encoder_count = (uint16_t)TIM3->CNT;    // Read latest HW counter value
}
uint32_t getSysTickCount(){
    // Return the value of the global counter variable.
    // The value is incremented by Systick
    return counter;
}

////////////////////////////////////////////////////////////////////////////////
// WS2812 Functions
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

////////////////////////////////////////////////////////////////////////////////
// Common Program Functions

void PC13_LED_Setup(){
    // Configure PC13 LED Indicator
    // - General Purpose Output Push-Pull
    // - Max Speed 50MHz
    RCC->APB2ENR |= 0x00000010;     // Enable I/O Clock for PortC
    GPIOC->CRH &= 0xFF0FFFFF;       // Zero Settings for PC13, preserve the rest
    GPIOC->CRH |= 0x00300000;       // Apply Config to PC13 (50MHz)
}
void toggleLed(){
    // Toggle the LED attached to PC13
    if (flash == 0){
        // Get LED Status
        if (GPIOC->ODR & 0x00002000){ // If Set
            GPIOC->BSRR |= 0x20000000; // Reset
            setAllRGB(0,0,0,pixels);                                            // DEBUG
        }else{                        // If Not Set
            GPIOC->BSRR |= 0x00002000; // Set
            setAllRGB(128,0,0,pixels);                                          // DEBUG
        }
        flash = 1001;
        writeLED(pixels,NUM_LEDS, DMA_Buffer);                                 // DEBUG
        
    }
}
void SysTick_Handler(void){
    counter++;
    
    // Log button state to debounce history
    update_button(&button_history, readButtonPB6); 
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
    
    if (idle != 0){
        idle--; // Decrement Idle Counter
    }
}
