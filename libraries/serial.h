// serial.h
#ifndef SERIAL_H
#define SERIAL_H

#include "serial.h"
#include "stm32f103xb.h" // Need as direct reference to HW
#include "buffer.h"      // Uses circular buffers.
#include <stdint.h> // uint8_t

// Currently this is written for USART1 only


void SerialSendByte(uint8_t);

uint8_t SerialReadByte();

void SerialBufferSend(volatile struct Buffer *serial_tx_buffer);

void SerialSendString(uint8_t *array, uint8_t array_length);

// SerialReceiveEcho();
// --------------------
// Checks if RXNE flag in USART1->SR, read data if set.
// Echoes data back to sending terminal.
void SerialReceiveEcho();

void SerialSetup(); // Setup USART1 & IRQ Priority
#endif // SERIAL_H
