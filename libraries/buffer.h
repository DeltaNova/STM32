// buffer.h - Circular Buffer

#ifndef BUFFER_H
#define BUFFER_H

#include "buffer.h"
#include <stdint.h> // uint8_t
#include <iostream> // std::cout
// Data Structure of Buffer
struct Buffer {
    uint8_t data[BUFFER_SIZE];
    uint8_t newest_index;
    uint8_t oldest_index;
};
/*
    TODO: The Buffer structure as it stands is a fixed size. It might be that a
    larger or smaller buffer is suitable for a particular task.

    If another struct is defined for a buffer of a different size then an
    instance of a buffer based on that size can be used instead.

    It is also the case the current Buffer structure is only capable of holding
    uint_8 values. The bufferWrite() & bufferRead() functions also only work on
    this datatype.

    DEV NOTE: Would it be a better idea in practice to define variable buffer
    sizes in the main program code rather than in a library? Or does it make
    sense to define say a small, standard, large Buffer struct and then allow
    the user to define additional sizes as required?
*/
uint8_t bufferWrite(volatile struct Buffer *buffer, uint8_t byte);
uint8_t bufferRead(volatile struct Buffer *buffer, uint8_t *byte);
uint8_t bufferPeek(volatile struct Buffer *buffer, uint8_t *byte);
uint8_t LoadBuffer(volatile struct Buffer *loadbuffer, uint8_t *data_array, uint8_t array_size);

#endif // BUFFER_H
