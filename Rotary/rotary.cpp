// rotary.cpp
// Program to use a rotart encoder
//  
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>             // Enable fixed width integers
#include <stdio.h>              // Newlib-nano
#include "clock.h"              // Setup system and peripheral clocks
#include "buffer_class.h"       // Cicular Buffers
#include "serial.h"             // USART1 Setup & Functions
#include "stm32f103xb.h"        // HW Specific Header
#include "systick.h"            // SysTick Configuration
extern volatile uint32_t ticks; // SysTick Library
////////////////////////////////////////////////////////////////////////////////
// Global Variables
volatile uint32_t flash = 0;        // Used for PC13 LED Flash Toggle Interval
////////////////////////////////////////////////////////////////////////////////
// Debugging Flags
//#define ENABLE_DEBUG_BUTTON_ACTION // Comment out to disable debugging
////////////////////////////////////////////////////////////////////////////////
// Function Declarations
extern "C" void USART1_IRQHandler(void);
extern "C" void SysTick_Handler(void);
void toggleLed();
void PC13_LED_Setup(); // Setup PC13 for output LED
void EncoderSetup();
void EncoderButtonSetup();
uint32_t get_upcounting_delta(uint32_t start_count, uint32_t stop_count);
uint16_t get_diff(uint16_t count, uint16_t last_count);
void update_encoder_counts();
uint8_t is_button_up(uint32_t *button_history);
uint8_t is_button_down(uint32_t *button_history);
uint8_t is_button_pressed(uint32_t *button_history);
uint8_t is_button_released(uint32_t *button_history);

// Struct to hold a value with range limits
struct Value {
    uint8_t value = 0;
    uint8_t valueMin = 0;
    uint8_t valueMax = 255;
};
void updateValue(Value &value, uint16_t dir, uint16_t delta);
void ValueShow(Value& value, Serial& serial, char *char_buffer);
void ValueShowMenu(Value& value, Serial& serial, char *char_buffer);

void processMenuSelection(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue);
// USART1
Buffer serial_tx; // USART1 TX Buffer (16 bytes)
Buffer serial_rx; // USART1 RX Buffer (16 bytes)

// Systick Counter
volatile uint32_t counter = 0;

// Rotary Encoder
static uint16_t encoder_count = 0;
static uint16_t last_encoder_count = 0; 

// Rotary Encoder Button
static uint16_t buttonPressStart = 0;
static uint16_t buttonPressStop = 0;
void buttonAction(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue);
void pressShort(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue);
void pressLong(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue);
void pressVlong(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue);
void showMenu(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue);


// Button Debounce 
void update_button(uint32_t *button_history);

// BUTTON_MASK - Assuming 1ms history update allows for 16ms of switch bounce.
#define BUTTON_MASK 0b11111111000000000000000011111111
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
int main(void) {
    ClockSetup();       // Setup System & Peripheral Clocks
    SysTick_Init();     // Enable SysTick
    
    // Create instance of Serial class (USART1) 
    Serial serial(serial_rx, serial_tx);     
    serial.setup();     // Enable Serial Support - Currently USART1 Specific
    // NOTE: Tie USART RX Pin low as I suspect interrupt causing problems.

    PC13_LED_Setup();   // Setup PC13 for output LED
    EncoderSetup();     // Setup Rotary Encoder
    EncoderButtonSetup(); // Setup the Rotary Encoder Button
    
    // NOTE: char_buffer needs to be sized to be able to hold the longest message.
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
        buttonAction(serial, char_buffer,MenuSelection,Red,Green,Blue);
        
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
            uint16_t delta = get_diff(encoder_count,last_encoder_count);
            
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

            // Output Backspaces to clear previous value before overwriting.
            for (uint8_t i=0;i<3; i++){
                serial.write(0x08);
            }  

            // Output Updated Value
            ValueShowMenu(V, serial, char_buffer);

        }else{  // No change in encoder count
            if (is_button_down(&button_history)){
                // Button Down - User interraction
                idle = TIMEOUT;             
            }   // Else No user Interraction
        }
    }
}

void showMenu(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue){
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
void pressShort(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue){
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

void processMenuSelection(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue){
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

void pressLong(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue){
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
            serial.write(0x5A);  // Z
            serial.newline();
            state = State::RED;
            break;
        case State::GREEN:
            // Zero Green Value, but remain in G
            Green.value = Green.valueMin; 
            serial.write(0x5A);  // Z
            serial.newline();
            state = State::GREEN;
            break;
        case State::BLUE:
            // Zero Blue Value, but remain in B
            Blue.value = Blue.valueMin; 
            serial.write(0x5A);  // Z
            serial.newline();
            state = State::BLUE;
            break;
        default:
            pressShort(serial, char_buffer,MenuSelection, Red, Green, Blue); // Default to pressShort()
            break;
    }
    
}

void pressVlong(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue){
    // Very Long Button Press Event
    // Zero All Colour Values and return to Idle state.
    Red.value = Red.valueMin;
    Green.value = Green.valueMin;
    Blue.value = Blue.valueMin;
    showMenu(serial,char_buffer,MenuSelection, Red,Green,Blue);
    state = State::IDLE;
    
    //TODO: I plan to use a SET state to turn LED's on, UNSET here to turn off.
}
uint32_t read_button(void){
    // Read the button state - Return 1 for pressed, 0 for released.
    uint32_t button_state;
    if (GPIOB->IDR & 0x00000040){   // PB6 High (Not Pressed)
        button_state = 0;
    }else{                          // PB6 Low (Pressed)
        button_state = 1;
    }
    return button_state;
}
void update_button(uint32_t *button_history){   // Called by Systick every 1ms
    // Bit shift button history to make room for new reading.
    *button_history = *button_history << 1;
    // Read current button state into history.
    *button_history |= read_button();
}
uint8_t is_button_up(uint32_t *button_history){
    // Returns true if button up
    return(*button_history == 0x00000000); 
}
uint8_t is_button_down(uint32_t *button_history){
    // Returns true if button down
    return(*button_history == 0xFFFFFFFF);
}
uint8_t is_button_pressed(uint32_t *button_history){
    // Returns true if pressed
    uint8_t pressed = 0;
    if ((*button_history & BUTTON_MASK) == 0b00000000000000000000000011111111){
        pressed = 1;
        // Set History to prevent retriggering from same press
        *button_history = 0xFFFFFFFF; 
    }
    return pressed;
}
uint8_t is_button_released(uint32_t *button_history){
    // Returns true if released
    uint8_t released = 0;
    if ((*button_history & BUTTON_MASK) == 0b11111111000000000000000000000000){
        released = 1;
        // Clear Histtory to prevent retriggering from same release
        *button_history = 0x00000000; 
    }
    return released;
}
void buttonAction(Serial& serial, char *char_buffer, Value &MenuSelection, Value &Red, Value &Green, Value &Blue){ 
    // Button Action from Polling
    // Parameters: Reference to USART instance, Pointer to char_buffer array

    #ifdef ENABLE_DEBUG_BUTTON_ACTION    
    static uint8_t buttonMessage[]= "Button Pressed\n\r"; //Size 16
    static uint8_t buttonMessage2[]= "Short Press\n\r"; //Size 13
    static uint8_t buttonMessage3[]= "Long Press\n\r"; //Size 12
    static uint8_t buttonMessage4[]= "Very Long Press\n\r"; //Size 17
    static uint8_t buttonMessage5[]= "Button Released\n\r"; //Size 17
    #endif
    if (is_button_pressed(&button_history)){
        // Reset Press Duration Counters to current counter value
        buttonPressStart = counter;
        buttonPressStop = counter;
        #ifdef ENABLE_DEBUG_BUTTON_ACTION
        
        serial.write_array(buttonMessage,16); // Send Pressed Message
        serial.write_buffer();
        #endif
    }
    if (is_button_released(&button_history)){
        buttonPressStop = counter;
        uint32_t buttondelta = get_upcounting_delta(buttonPressStart, buttonPressStop);
        #ifdef ENABLE_DEBUG_BUTTON_ACTION
        // Send Released Message
        serial.write_array(buttonMessage5,17);
        serial.write_buffer();
        #endif
        // Check press time and select the message to load into the buffer.
        if (buttondelta < 1000){                        // Short Press
            #ifdef ENABLE_DEBUG_BUTTON_ACTION
            serial.write_array(buttonMessage2,13);
            #endif
            pressShort(serial, char_buffer, MenuSelection, Red, Green, Blue);
        }else if (buttondelta <5000){                   // Long Press
            #ifdef ENABLE_DEBUG_BUTTON_ACTION
            serial.write_array(buttonMessage3,12);
            #endif
            pressLong(serial, char_buffer, MenuSelection, Red, Green, Blue);
        }else{                                          // Very Long Press
            #ifdef ENABLE_DEBUG_BUTTON_ACTION
            serial.write_array(buttonMessage4,17);
            #endif
            pressVlong(serial, char_buffer, MenuSelection, Red, Green, Blue);
        }
        #ifdef ENABLE_DEBUG_BUTTON_ACTION
        // Write the buffer containing the message to USART.
        serial.write_buffer();  
        
        // Debug Code to Print Length of button press
        snprintf(char_buffer, 12, "%010lu", buttondelta);
        for(uint8_t i=0;i<10; i++){
            serial.write(char_buffer[i]);
        }
        // Add a new line
        serial.write(0x0A); // LF
        serial.write(0x0D); // CR
        #endif

        
    }
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

uint32_t get_upcounting_delta(uint32_t start_count, uint32_t stop_count){
    // Returns the difference between two count values.
    
    // DEV NOTE: Count values need to be be from an upcounting only counter.
    //           Decrementing counts can result in large deltas where only a
    //           small delta exits in reality.
    
    if (stop_count < start_count){
        // Counter has rolled over (at least once, multi rollover not handled)
        return ((0xFFFFFFFF - start_count) + 1 + stop_count);
    }else{
        return (stop_count - start_count);
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
uint16_t get_diff(uint16_t count, uint16_t last_count){
    // This function assumes a count range of 0x0000 to 0xFFFF with roll over
    // and roll under.
    
    // Note: It will not work correctly if the count is scaled before use.
    
    // A window is used to assess the difference between the count and 
    // last_count values. Values inside or outside of the window determine how 
    // the diff is calculated. Multiple rollover not handled.
    uint16_t diff;
    if (count > last_count){
        if (count < 0x8000){
            return (count - last_count);            // Increment
        }else{ // (count >= 0x8000)
            if (last_count > (count-0x4000)){
                return (count - last_count);        // Increment
            }else{ // (last_count <= (count-0x4000))
                // Count Roll Under Condition
                diff = (0xFFFF - count);
                return(last_count + diff);          // Decrement
            }
        }
    }else{ // (count <= last count)
        if (last_count < 0x8000){
            return(last_count - count);             // Decrement
        }else{ // (last_count >= 0x8000)
            if (count > (last_count - 0x4000)){
                return(last_count - count);         // Decrement
            }else{ //(count <= (last_count - 0x4000))
                // Count Roll Over Condition
                diff = 0xFFFF - last_count;
                return (count + diff);              // Increment
            }
        }
    }
}
void update_encoder_counts(){
    // Read the hardware counter connected to the rotary encoder.
    last_encoder_count = encoder_count;     // Store previous encoder_count 
    encoder_count = (uint16_t)TIM3->CNT;    // Read latest HW counter value
}
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
        }else{                        // If Not Set
            GPIOC->BSRR |= 0x00002000; // Set
        }
        flash = 1001;
    }
}
void SysTick_Handler(void){
    counter++;
    update_button(&button_history); // Log button state to debounce history
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
