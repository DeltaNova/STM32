// Basic STM32 demo that runs from the internal oscillator,
// flashes an LED and writes out serial data.

#include "stm32f103xb.h"

#include <stdio.h> // Provides printf (Semihosting)

#ifdef DEBUG
extern void initialise_monitor_handles(void);
#endif
/// Spin delay
void delay(int count)
{
    // volatile so that the compiler doesn't optimise it out
    volatile int i;

    for (i = 0; i < count; i++)
    {
    }
}

/// Main function.  Called by the startup code.
int main(void)
{
    #ifdef DEBUG
    initialise_monitor_handles();
    #endif
    // Most of the peripherals are connected to APB2.  Turn on the
    // clocks for the interesting peripherals
    RCC->APB2ENR = 0
        // Turn on USART1
        | RCC_APB2ENR_USART1EN
        // Turn on IO Port A
        | RCC_APB2ENR_IOPAEN
        // Turn on IO Port B
        | RCC_APB2ENR_IOPBEN
        // Turn on the alternate function block
        | RCC_APB2ENR_AFIOEN;

    // Put Port A pins 8 through 15 into alternate function/50 MHz
    // mode.
    GPIOA->CRH = 0xBBBBBBBB;

    // Put PB0 into push pull 50 MHz mode
    GPIOB->CRL = 0x03;

    // The internal RC oscillator runs at 8 MHz.  The divide by 16
    // combined with the fractional divider means we just divide the
    // bus clock by the baud rate
    USART1->BRR = 8000000/38400;
    // Enable the UART, TX, and RX circuit
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    // Nothing else /needs/ to be setup but you could go further.  The
    // chip defaults to 8 MHz RC with the busses running at the same
    // speed as the core.

    for (;;)
    {
        // Around 1/4 of a second = 100000
        delay(3200000);

        // Write out a character
        USART1->DR = 'H';
        // Turn all pins on
        GPIOB->ODR = ~0;

        delay(1600000);

        // Write out another.  Note that the delay takes long enough
        // so we know that TX is empty
        USART1->DR = 'i';
        // Turn all pins off
        GPIOB->ODR = 0;

        #ifdef DEBUG
        //Test SemiHosting
        //setbuf(stdout, NULL);
        printf("Hello\n");
        #endif
    }
}
