// delay.cpp
#include <stdint.h>

void delay(int count){
    // volatile so that the compiler doesn't optimise it out
    volatile int i;
    for (i = 0; i < count; i++){
    }
}

void longdelay(uint32_t count){
    volatile uint32_t i;
    for (i = 0; i < count; i++){
    }
}
