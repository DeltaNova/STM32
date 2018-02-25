/*******************************************************************************
  * system_stm32f1xx.c
  *  - SystemInit(): Setups the system clock (System clock source,
  *                  PLL Multiplier factors, AHB/APBx prescalers and Flash
  *                  settings).
  *                  This function is called at startup just after reset and
  *                  before branch to main program. This call is made inside
  *                  the "startup_stm32f1xx_xx.s" file.
  *
  *  On device reset the HSI (8 MHz) is used as system clock source.
  *  Then SystemInit() function is called by "startup_stm32f1xx_xx.s" file,
  *  to configure the system clock before to branch to main program.
  *****************************************************************************/
#include "stm32f1xx.h"

/* Vector Table Offset */
/* Ref: PM0056 - STM32F10xxx CM3 Prog.Man. SCB_VTOR Register                  */
#define VECTOR_TABLE_OFFSET  0x0

/*******************************************************************************
*  Clock Definitions
*******************************************************************************/
void SystemInit (void)                   /* Initialise System Following Reset */
{
  /* Ref: RM0008 - STM32F1xxxx Ref. Manual. RCC_CR, RCC_CFGR, RCC_CIR         */
  /* Reset the RCC clock config to the default reset state(for debuging)      */
  RCC->CR |= (uint32_t)0x00000001;    /* Set HISON bit, Turns on 8MHz Int Osc */

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits                        */
  /* SW     = 0b00   - Select HSI (High Speed Internal Osc) as system clock   */
  /* HPRE   = 0b0000 - AHB Prescaller = System Clk /0 (Results in HSI@8MHz)   */
  /* PPRE1  = 0b000  - APB Low-Speed Prescaler  (APB1) = HCLK/0               */
  /* PPRE2  = 0b000  - APB High-Speed Prescaler (APB2) = HCLK/0               */
  /* ADCPRE = 0b00   - ADC Prescaller = PCLK/2                                */
  /* MCO    = 0b000  - Microcontroller Clock Output = No Clock                */
  RCC->CFGR &= (uint32_t)0xF8FF0000;

  /* Reset HSEON, CSSON and PLLON bits */
  /* HSEON = 0b0 - Turns off HSE (High Speed Ext. Osc.)                       */
  /* CSSON = 0b0 - Turns off CSS (Clock Security System)                      */
  /* PLLON = 0b0 - Turns off PLL (Phase Locked Loop)                          */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset HSEBYP bit */
  /* HSEBYP = 0b0 - Turn off HSEBYP (High Speed External Clock Bypass)        */
  /* HSEBYP bit can only be written if HSE disabled (i.e. HSEON = 0)          */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  /* PLLSRC   = 0b0        - PLL Input clock (High Speed Int.Osc /2)          */
  /* PLLXTPRE = 0b0        - HSE Divider for PPL Entry - Not divided          */
  /* PLLMUL   = 0b0000     - PLL Multiplier (* 2)                             */
  /* USBPRE/OTGFSPRE = 0b0 - USB Prescaller (PLL Clk / 1.5)                   */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;

  /* Clock Interrupt Register - Disable all interrupts and clear pending bits */
  RCC->CIR = 0x009F0000;

  /* Vector Table Relocation in Internal FLASH. */
  /* Relocating the vector table seems to be useful if you are using a
     bootloader. Until there is a reason to move it leave it where it is      */
  /* Ref: PM0056 - STM32F10xxx CM3 Prog.Man. SCB_VTOR Register                */
  SCB->VTOR = FLASH_BASE | VECTOR_TABLE_OFFSET;
}
/******************************************************************************/
