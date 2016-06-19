// buffer.cpp - Circular Buffer Example

/*
  The MIT License (MIT)

  Original work Copyright (c) 2015 Elliot Williams
  Modified work Copyright (c) 2016 Matthew Tunstall

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
/*
    Changes:
    June 2016 M.Tunstall - Modified example based on these original works
        https://hackaday.com/2015/10/29/embed-with-elliot-going-round-with-circular-buffers/
        https://github.com/hexagon5un/embed_with_elliot-circular_buffer
*/

#define BUFFER_SIZE  16

//#include <stdint.h> // uint8_t
#include <iostream> // std::cout

/*
    COMPILE:
        g++ -std=gnu++11 -o buffer buffer.cpp

    USAGE:
        ./buffer

    EXPECTED OUTPUT:
        This is a test of a circular buffer.
        We will start by storing a message in the buffer
        We now have data in the buffer, time to read it
        Print Me Later
*/


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
// Data Structure of Buffer
struct Buffer {
    uint8_t data[BUFFER_SIZE];
    uint8_t newest_index;
    uint8_t oldest_index;
};

uint8_t bufferWrite(volatile struct Buffer *buffer, uint8_t byte){
    // Pointer to Buffer Structure passed to function as argument.
    // volatile because the buffer needs to be updated via interrupts.
    // Ref: http://hackaday.com/2015/08/18/embed-with-elliot-the-volatile-keyword/

    /*
    Ref:
    www.hackaday.com/2015/10/29/embed-with-elliot-going-round-with-circular-buffers/

    "Now let’s write some data into the buffer. To do so, we first check to see
    if we’ve got space for a new entry: that is, we make sure that the
    newest_index+1 doesn’t run into the oldest_index. If it does, we’ll want to
    return a buffer full error. Otherwise, we just store the data in the next
    slot and update the newest_index value."

    "Note that when we’re calculating the next_index, we take the modulo with
    BUFFER_SIZE. That does the “wrapping around” for us. If we have a 16-byte
    buffer, and we are at position fifteen, adding one gives us sixteen, and
    taking the modulo returns zero, wrapping us around back to the beginning."
    */
    uint8_t next_index = (((buffer->newest_index)+1) % BUFFER_SIZE);

    // Is buffer full?
    if (next_index == buffer->oldest_index){
        return 2; // Buffer Full
    }
    // Buffer has space, continue.

    // Store byte at the free location in the buffer.
    buffer->data[buffer->newest_index] = byte;

    // Update newest_index to the next data location.
    buffer->newest_index = next_index;

    return 0; //Buffer OK
}

uint8_t bufferRead(volatile struct Buffer *buffer, uint8_t *byte){
    // Pointer to Buffer Structure passed to function as argument.
    // volatile because the buffer needs to be updated via interrupts.
    // Ref: http://hackaday.com/2015/08/18/embed-with-elliot-the-volatile-keyword/
    // Pointer to location for read byte to be stored passed as argument,
    // required as return value is for buffer status.

    // Is the buffer empty?
    if (buffer->newest_index == buffer->oldest_index){
        return 1; //Buffer Empty
    }

    // Buffer contains data, continue.

    // Read the oldest data in the buffer and store it at the byte location
    // being pointed to.
    *byte = buffer->data[buffer->oldest_index];

    // Update the the oldest_index to the oldest unread data.
    buffer->oldest_index = ((buffer->oldest_index+1) % BUFFER_SIZE);

    return 0; //Buffer OK
}

uint8_t bufferPeek(volatile struct Buffer *buffer, uint8_t *byte){
    /*
    One trick is in defining the last_index. When the buffer has just wrapped
    around, newest_index can be equal to zero. Subtracting one from that (to go
    back to the last written character) will end up with a negative number in
    that case. Adding another full BUFFER_SIZE to the index prevents the value
    from ever going negative, and the extra factor of BUFFER_SIZE gets knocked
    out by the modulo operation anyway.
    */
    uint8_t last_index = ((BUFFER_SIZE + (buffer->newest_index) - 1) % BUFFER_SIZE);

    // Is the buffer empty?
    if (buffer->newest_index == buffer->oldest_index){
        return 1; // Buffer Empty
    }

    *byte = buffer->data[last_index];
    return 0; // Buffer OK
    }


int main(void){
    std::cout << "This is a test of a circular buffer.\n";
    // Begin by defining an instance of the Buffer called buffer.
    // No Data, newest_index = 0, oldest_index = 0
    volatile struct Buffer buffer {{},0,0};

    // This is the data to put into the buffer.
    uint8_t data2store [] = "Print Me To Screen Later\n";

    std::cout << "We will start by storing a message in the buffer\n";

    // Zero Counter
    uint8_t i = 0;

    // While there continues to be data to store in the buffer.
    while(i < sizeof(data2store) - 1){

        // TODO: Test for buffer full
        // Store data byte in buffer unless it is full
        if(bufferWrite(&buffer, data2store[i]) == 0){
            // Increment counter
            i++;
        }else{
            std::cout << "Buffer is Full!\n";
            break;
        }
    }

    std::cout << "We now have data in the buffer, time to read it\n";

    uint8_t tempCharStorage; // Location in memory to store the read byte
    // While there is data in the buffer
    while(bufferRead(&buffer, &tempCharStorage) == 0){
        std::cout << tempCharStorage;
    }
    return 0;
}
