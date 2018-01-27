// i2c.cpp -- I2C Setup STM32F103

#include "stm32f103xb.h" // Need as direct reference to HW
#include "i2c.h"         // Library Header
#include "buffer.h"     // Buffer Library

Status I2C1Setup() {
    // Ref: Datasheet DS5319 Section 5.3.16 I2C Interface Characteristics
    // Ref: STM32F10xx8 STM32F10xxB Errata sheet Rev 13 Section 2.13.7
    // Note: Incorporates workaround for locking BUSY flag detailed in errata.
    
    // Setup Required Clocks
    // ABP1 Bus Speed should be 36MHz
    
    // Use Alternate Function Remapped ports rather than the defaults to avoid
    // the port conflict with USART1.
    // I2C1 SDA - PB7 (Alternate Function Default) [Conflict with USART1_RX]    // TODO: Make I2C1Setup accept a variable to switch pin mapping.
    // I2C1 SCL - PB6 (Alternate Function Default) [Conflict with USART1_TX]    //       This will require changes to the pin specific workarounds.
    
    // I2C1 SDA - PB9 (Alternate Function Remap)
    // I2C1 SCL - PB8 (Alternate Function Remap)

    RCC->APB2ENR |= 0x00000009;     // Enable PortB Clock & Alt. Function Clk
    RCC->APB1ENR |= 0x00200000;     // Enable I2C1 Peripheral Clock
    
    // Workaround Step 1
    // Disable the I2C Peripheral by clearing the PE bit in I2Cx_CR1 register
    I2C1->CR1 &= ~0x0001;           // Disable I2C Peripheral 
    
    // Workaround Step 2
    // Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, 
    // High level (Write 1 to GPIOx_ODR)
    
    GPIOB->CRH &= ~0x000000F0;      // Reset PB9 bits (SDA)
    GPIOB->CRH |= 0x00000070;       // General Ouput, Open Drain
    GPIOB->CRH &= ~0x0000000F;      // Reset PB8 bits
    GPIOB->CRH |= 0x00000007;       // General Output, Open Drain
    GPIOB->ODR |= 0x00000300;       // Set PB8,PB9 High Level Output
    
    // Workaround Step 3
    // Check SCL and SDA High level in GPIOx_IDR.
    while(!(GPIOB->IDR & 0x00000300)); // Check that PB8, PB9 are High level.
    
    // Workaround Step 4
    // Configure the SDA I/O as General Purpose Output Open-Drain,
    // Low level (Write 0 to GPIOx_ODR).
    GPIOB->ODR &= ~0x00000200;      // Set PB9 (SDA) Low Level.
    
    // Workaround Step 5
    // Check SDA Low level in GPIOx_IDR.
    while(GPIOB->IDR & 0x00000200); // Check that PB9 is low level.
    
    // Workaround Step 6
    // Configure the SCL I/O as General Purpose Output Open-Drain,
    // Low level (Write 0 to GPIOx_ODR).
    GPIOB->ODR &= ~0x00000100;      // Set PB8 Low
  
    // Workaround Step 7
    // Check SCL Low level in GPIOx_IDR.
    while(GPIOB->IDR & 0x00000100); // Check that PB8 is Low.

    // Workaround Step 8
    // Configure the SCL I/O as General Purpose Output Open-Drain,
    // High level (Write 1 to GPIOx_ODR).
    GPIOB->ODR |= 0x00000100;       // Set PB8 High

    // Workaround Step 9
    // Check SCL High level in GPIOx_IDR.
    while(!(GPIOB->IDR & 0x00000100));  //Check that PB8 (SCL) is High

    // Workaround Step 10
    // Configure the SDA I/O as General Purpose Output Open-Drain,
    // High level (Write 1 to GPIOx_ODR).
    GPIOB->ODR |= 0x00000200;       // Set PB9 High

    // Workaround Step 11
    // Check SDA High level in GPIOx_IDR.
    while(!(GPIOB->IDR & 0x00000200));  // Check that PB9 (SDA) is High
    
    // Workaround Step 12
    // Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIOB->CRH |= 0x000000F0;       // AF Open Drain, Max 50MHz
    GPIOB->CRH |= 0x0000000F;       // AF Open Drain, Max 50MHz
    AFIO->MAPR |= 0x00000002;       // Remap I2C1 to use PB8,PB9
        
    // Workaround Step 13
    // Set SWRST bit in I2Cx_CR1 register.
    I2C1->CR1 |= 0x8000;            // Set SWRST

    // Workaround Step 14
    // Clear SWRST bit in I2Cx_CR1 register.
    I2C1->CR1 &= ~ 0x8000;          // Clear SWRST
    
    // Start I2C Configuration - After SWRST before PE = 1
    // I2C Master Mode
    I2C1->CR2 = 0x0024;             // FREQ of Peripheral Clk = 36MHz
    I2C1->CR2 |= 0x0100;            // Enable Error Interrupt

    // Setup Own Address Register
    I2C1->OAR1 |= 0x0400;           // Bit 14 needs to kept at 1 by software.
    
    // Configure Clock Control Reg
    // First ensure peripheral is disabled
    I2C1->CR1 &= 0xFFFE;            // PE = 0

    // Setup Rise Time
    // Fast Mode     - TRISE = 11 = 0x000B
    // Standard Mode - TRISE = 37 = 0x0025
    I2C1->TRISE = 0x000B;

    //  Ref: Datasheet Table 41 SCL Frequency
    //  400kHz = 0x801E - Fast Mode, Duty = 2, CCR = 30
    //  100kHz = 0x00B4 - Standard Mode
    //  400kHz = 0xC004 - Fast Mode, Duty = 16/9, CCR = 4
    I2C1->CCR = 0xC004;
    // End Additional Setup
    
    // Workaround Step 15
    // Enable the I2C peripheral by setting the PE bit in the I2Cx_CR1 register.
    I2C1->CR1 |= 0x0001;            // Enable I2C, PE = 1

    // Finalise Setup
    I2C1->CR1 &= 0xFBF5;            // Clear ACK, SMBTYPE and SMBUS bits
    // Only enable after all setup operations complete.
    I2C1->CR1 |= 0x0400;            // Acknowledge Enable - once PE = 1
    return Success;
}

/*void I2CStart()
{
    while(I2C1->SR2 & 0x0002); // Wait whilst busy
    // Set Start Condition by setting START bit
    // Master/Slave bit set once Busy bit clear.
    I2C1->CR1 |= 0x0100;
    // SB = 1, PE = 1

    // Wait for MSL = 1 (& Busy = 0)

    while(!(I2C1->SR2 & 0x0001));
}
*/
Status I2CWriteMode(uint8_t SlaveAddr)        // 7bit Addressing Mode             // TODO: Combine with I2CReadMode as almost identicle functions
{
    uint16_t Timeout = 0xFFFF;

    while(I2C1->SR2 & 0x0002);              // Wait whilst BUSY
    I2C1->CR1 |= 0x0100;                    // Set START bit
    // EV5 Start
    while((I2C1->SR1 & 0x0001) != 0x0001){  // Wait until start bit set
        if (Timeout-- == 0) {               // If timeout reached
            return Error;                   // Timeout Occured Return Error
        }
    }

    uint8_t Address = (SlaveAddr & 0xFE); // Clear SlaveAddr LSB for write mode
    I2C1->DR = Address;                   // Write Address to Data Register     
    // EV5 End

    // Read of SR1 and write to DR should have reset the start bit in SR1

    Timeout = 0xFFFF;
    // EV6 Start                                                                
    while(!(I2C1->SR1 & 0x0002)){// Read SR1, Wait for ADDR Flag Set
        if (Timeout-- == 0)
            return Error;                   // Timeout Occured Return Error
    }

    // Addr bit now needs to be reset. Read SR1 (above) then SR2
    (void)I2C1->SR2;
    // EV6 End
    // Write Mode Enabled. Send Data using I2CWriteData()
    return Success;
}

Status I2CWriteData(uint8_t Data)
{
    // Write Data Byte to established I2C Connection
    // Load Data Register with byte to send.
    I2C1->DR = Data;
    // Wait for Byte Transfer Finished (BTF) flag in I2C1->SR1
    uint16_t Timeout = 0xFFFF;
    while(!(I2C1->SR1 & 0x0004)){
        if (Timeout-- == 0)
            return Error;
    }
    return Success;
}

Status I2CStop()
{
    // End the I2C Communication Session

    // To close the connection a STOP condition is generated 
    // by setting the STOP bit.
    I2C1->CR1 |= 0x0200; // Set STOP bit

    // Check STOP bit has been cleared indicating STOP condition detected.
    uint16_t Timeout = 0xFFFF;
    while(!(I2C1->CR1 & 0x0200)){
        if (Timeout-- == 0)
            return Error;
    }
    // Communication Ended, I2C interface in slave mode.
    return Success;
}
/*
void I2CReadMode(uint8_t SlaveAddr)                                             // TODO: Combine with I2CWriteMode as almost identicle functions
{   // 7bit Addressing Mode

    while(!(I2C1->SR1 & 0x0001));   // Wait for start bit to be set
    SlaveAddr |= 0x0001;            // Set Slave Addr LSB to indicate read mode
    I2C1->DR = SlaveAddr;           // Write SlaveAddr to Data Register

    // Read of SR1 and write to DR should have reset the
    // start bit in SR1

    // Wait for confirmation that addr has been sent.
    // Check ADDR bit in I2C1->SR
    while(!(I2C1->SR1 & 0x0002)); // Read SR1

    // Addr bit now needs to be reset. Read SR1 (above) then SR2
    (void)I2C1->SR2; // Read SR2
    // Read Mode Enabled, Receive Data using I2CReadData()
}
*/

Status I2CReadByte(uint8_t SlaveAddr, volatile struct Buffer *i2c_rx_buffer){
    
    I2C1->CR1 &= ~(0x0400);         // Clear ACK Flag
    __disable_irq();                // Disable Interrupts

    // Clear Addr Flag
    while(!(I2C1->SR1 & 0X0002));   // Read SR1 & Check for Addr Flag
    (void)I2C1->SR2;                // Read SR2 to complete Addr Flag reset.

    I2C1->CR1 |= 0x0200;            // Set Stop Flag
    __enable_irq();                 // Enable Interrupts

    while(!(I2C1->SR1 & 0x0040));   // Wait for RxNE Set

    bufferWrite(i2c_rx_buffer, I2C1->DR); // Read Byte into Buffer

    while (I2C1->CR1 & 0x0200);     // Wait until STOP Flag cleared by HW
    I2C1->CR1 |= (0x0400);          // Set ACK
    return Success;
}

Status I2CRead2Bytes(uint8_t SlaveAddr, volatile struct Buffer *i2c_rx_buffer){
        I2C1->CR1 |= 0x0800;                    // Set POS Flag

        // EV6_1 - Disable Acknowledge just after EV6 after ADDR is cleared.
        //         Note: Disable IRQ around ADDR Clearing
        __disable_irq();                // Disable Interrupts
        // Complete Clear Addr Flag Sequence by reading SR2
        (void)I2C1->SR2;                // Read SR2, to reset ADDR Flag.

        I2C1->CR1 &= ~(0x0400);         // Clear ACK Flag
        __enable_irq();                 // Enable Interrupts

        while(!(I2C1->SR1 & 0x0004));   // Wait BTF Flag (Byte Transfer Finish)
        
        // Disable interrupts around STOP due to HW limitation
        __disable_irq();                // Disable Interrupts
        I2C1->CR1 |= 0x0200;            // Set Stop Flag
        bufferWrite(i2c_rx_buffer, I2C1->DR); // Read 1st Byte into Buffer
        __enable_irq();                 // Enable Interrupts
        bufferWrite(i2c_rx_buffer, I2C1->DR); // Read 2nd Byte into Buffer

        while (I2C1->CR1 & 0x0200);     // Wait until STOP Flag cleared by HW

        // Clear POS Flag, Set ACK Flag (to be ready to receive)
        I2C1->CR1 |= (0x0400);          // Set ACK
        I2C1->CR1 &= ~(0x0800);         // Clear POS
        return Success;
    }

Status I2CRead3Bytes(uint8_t SlaveAddr, uint8_t NumberBytesToRead, volatile struct Buffer *i2c_rx_buffer){

    uint16_t Timeout = 0xFFFF;
    // Clear Addr Flag
    while(!(I2C1->SR1 & 0X0002)){           // Read SR1 & Check for Addr Flag
        if (Timeout-- == 0){
            return Error;                   // Timeout occured return Error
        }
    }
    
    // Read SR2 to complete Addr Flag reset.
    (void)I2C1->SR2;                        // Read SR2
    
    while (NumberBytesToRead > 3){
        Timeout = 0xFFFF;
        while(!(I2C1->SR1 & 0x0004)){       // Wait until BTF = 1
            if (Timeout-- == 0){
                return Error;               // Timeout occured return Error
            }
        }
        
        // Read Byte in Data Register and store in Buffer
        bufferWrite(i2c_rx_buffer, I2C1->DR);  
        
        // Decrement NumberBytesToRead
        NumberBytesToRead = NumberBytesToRead - 1;
    }

    // At this point there are 3 bytes left to read.

    // Dev Note:
    // ---------
    // The sequence is detailed in RM0008 (Rev16)- Fig 274 and in
    // AN2824 (Rev4) - Fig 1. There is an error in the flowchart in Fig 1;
    // it doesnt read N-2 as the sequence EV7_2 in Fig 274 outlines.
    // The correct sequence is below.
    
    Timeout = 0xFFFF;
    while(!(I2C1->SR1 & 0x0004)){           // Wait until BTF Flag = 1
        if (Timeout-- == 0){
            return Error;                   // Timeout occured return Error
        }
    }
    
    I2C1->CR1 &= ~(0x0400);                 // Clear ACK Flag
    __disable_irq();                        // Disable Interrupts

    // Missing read from AN2824 (Rev4) Fig 1
    bufferWrite(i2c_rx_buffer, I2C1->DR);   // Read Byte N-2 into Buffer
    I2C1->CR1 |= 0x0200;                    // Set Stop Flag
    bufferWrite(i2c_rx_buffer, I2C1->DR);   // Read Byte N-1 into Buffer
    __enable_irq();                         // Enable Interrupts
    
    Timeout = 0xFFFF;
    while(!(I2C1->SR1 & 0x0040)){           // Wait for RxNE Set
        if (Timeout-- == 0){
            return Error;                   // Timeout occured return Error
        }
    }
    
    bufferWrite(i2c_rx_buffer, I2C1->DR);   // Read Byte N into Buffer
    NumberBytesToRead = 0;                  // All Bytes Read
    
    Timeout = 0xFFFF;
    while (I2C1->CR1 & 0x0200){             // Wait for STOP Flag HW clear
        if (Timeout-- == 0){
            return Error;                   // Timeout occured return Error
        }
    }
    I2C1->CR1 |= (0x0400);                  // Set ACK
    return Success;
}

Status I2CReadData(uint8_t NumberBytesToRead, uint8_t SlaveAddr, volatile struct Buffer *i2c_rx_buffer )
{
    // Dev Note:
    // There are a number of calls to enable/disable interrupts in this section.
    // This is related to limitations in the silicon errata sheet.
    // ES096 - STM32F10xx8 (Rev12)  Which covers F10xx8/B devices.
    // Details: Section 2.13.2, Workaround 2.

                                                                                // TODO: Add a check to see if NumberBytesToRead > Buffer to prevent overflow.
    // NOTE: The buffer size is set globally for all buffers.
    // Implementation Based on Application Note AN2824

    // If nothing to read return straight away.
    if (NumberBytesToRead == 0){
        return Success;
    }

    uint16_t Timeout = 0xFFFF;
    while(I2C1->SR2 & 0x0002){              // Wait whilst BUSY
        if (Timeout-- == 0){
            return Error;                   // Timeout occured return Error
        }
    }
    
    Timeout = 0xFFFF;
    I2C1->CR1 |= 0x0100;                    // Set START bit
    // EV5 Start
    while((I2C1->SR1 & 0x0001) != 0x0001){  // Wait until start bit set
        if (Timeout-- == 0){
            return Error;                   // Timeout occured return Error
        }
    }

    SlaveAddr |= 0x0001;         // Set Slave Addr LSB to indicate read mode
    I2C1->DR = SlaveAddr;        // Write SlaveAddr to Data Register


    Timeout = 0xFFFF;
    // EV6 Start
    while(!(I2C1->SR1 & 0x0002)){           // Read SR1, Wait for ADDR Flag Set
        if (Timeout-- == 0){                 
            return Error;                   // Timeout occured return Error
        }
    }

    if (NumberBytesToRead == 1){
        I2CReadByte(SlaveAddr, i2c_rx_buffer);
    } else if (NumberBytesToRead == 2) {
        I2CRead2Bytes(SlaveAddr, i2c_rx_buffer);
    } else {    // Read 3+ Bytes
        I2CRead3Bytes(SlaveAddr, NumberBytesToRead, i2c_rx_buffer);
    }
    return Success;
}
