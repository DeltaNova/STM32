/*******************************************************************************
  * system_stm32f1xx.h - Header for custom system file
  *****************************************************************************/
  
/* Define to prevent recursive inclusion                                      */
#ifndef __SYSTEM_STM32F10X_H
#define __SYSTEM_STM32F10X_H

extern "C" {
extern uint32_t SystemCoreClock;       /* System Clock Frequency (Core Clock) */
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
}

#endif /*__SYSTEM_STM32F10X_H */

