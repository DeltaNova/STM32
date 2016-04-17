// Basic STM32 demo that runs from the HSI (High Speed Internal) oscillator on
// power up and switches over to use the HSE (High Speed External) oscillator.

#include "stm32f103xb.h"

#include <stdio.h> // Provides printf (Semihosting)

#ifdef DEBUG
extern void initialise_monitor_handles(void);
#endif

void semihost(void) {
    // Initialise Semihost requirement based on DEBUG flag setting
    #ifdef DEBUG
    initialise_monitor_handles();
    #endif
}

void semihostmsg(uint8_t msg) {
    // Output Selected Semihost message
    #ifdef DEBUG
    switch(msg){
    case 0:
        printf("Hello World\n");
        break;
    default:
        printf("Invalid Message\n");
    }
    #endif
}

// Main - Called by the startup code.
int main(void) {
    semihost(); // Add semihost functionality

    semihostmsg(0); // Send Semihost Message
    semihostmsg(1); // Send Semihost Message
}
