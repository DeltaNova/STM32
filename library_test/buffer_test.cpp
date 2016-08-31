// buffer_test.cpp - Circular Buffer Example testing buffer.h/buffer.cpp
#include "buffer.h"
#include <stdint.h> // uint8_t
#include <iostream> // std::cout
/*
    COMPILE
    -------
    g++ -std=gnu++11 -L buffer -W -o buffer_test buffer_test.cpp buffer.cpp
*/

int main(void){
    std::cout << "This is a test of a circular buffer.\n";
    // Begin by defining an instance of the Buffer called buffer.
    // No Data, newest_index = 0, oldest_index = 0
    volatile struct Buffer buffer {{},0,0};

    // This is the data to put into the buffer.
    uint8_t data2store [] = "Print Me To Screen Later\n";
    ////////////////////////////////////////////////////////////////////////////
    // TEST 1
    std::cout << "Test 1: We will start by storing a message in the buffer\n";

    LoadBuffer(&buffer, data2store, sizeof(data2store)); // Put data into the buffer

    ////////////////////////////////////////////////////////////////////////////
    // TEST 2
    std::cout << "Test 2: We now have data in the buffer, time to read it\n";
    uint8_t tempCharStorage; // Location in memory to store the read byte
    // While there is data in the buffer
    while(bufferRead(&buffer, &tempCharStorage) == 0){
        std::cout << tempCharStorage;
    }
    // The buffer is now empty.

    ////////////////////////////////////////////////////////////////////////////
    // TEST 3
    std::cout << "\nTest 3: Try out bufferPeek()\n";

    uint8_t check_byte; // Storage for output of bufferPeek()
    // Lets add some data to the buffer.

    // This is the data to put into the buffer.
    uint8_t data2store2 [] = "Hello\n";
    // Zero Counter
    uint8_t j = 0;

    // While there continues to be data to store in the buffer.
    while(j < sizeof(data2store2) - 1){
        // Store data byte in buffer unless it is full
        if(bufferWrite(&buffer, data2store2[j]) == 0){
            // Increment counter
            j++;
            // Obtain the buffer status and store the last byte as check_byte
            uint8_t buffer_status = bufferPeek(&buffer, &check_byte);
            if ( buffer_status == 0 && check_byte == 'l'){
            std::cout << "The letter 'l' has been stored in the buffer\n";
            }

        }else{
            std::cout << "Buffer is Full!\n";
            break;
        }
    }

    return 0;
}


