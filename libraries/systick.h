// systick.h - Project Specific Systick Header
#ifndef SYSTICK_H
#define SYSTICK_H
// ticks is required to be accessable by the Systick Hander in the main program.
volatile uint32_t ticks = 0;        // Used for SysTick count down.
void SysTick_Init(void);
void delay_ms(uint32_t ms);
#endif //SYSTICK_H
