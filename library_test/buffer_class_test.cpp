// buffer_test.cpp - Circular Buffer Example testing buffer.h/buffer.cpp
#include "../libraries/buffer_class.h"
#include <stdint.h> // uint8_t
#include <iostream> // std::cout
/*
    COMPILE
    -------
    g++ -std=gnu++11 -L buffer_class -W -o buffer_class_test buffer_class_test.cpp buffer_class.cpp
*/

int main(void){
    std::cout << "This is a test of a circular buffer.\n";
    // Begin by defining an instance of the Buffer called buffer.
    // No Data, newest_index = 0, oldest_index = 0
    //volatile struct Buffer buffer {{},0,0};
    Buffer buffer;

    // This is the data to put into the buffer.
    uint8_t data2store [] = "Print Me To Screen Later\n";
    ////////////////////////////////////////////////////////////////////////////
    // TEST 1
    std::cout << "Test 1: We will start by storing a message in the buffer\n";
    buffer.load(data2store, sizeof(data2store)); // Put data into the buffer
    
    ////////////////////////////////////////////////////////////////////////////
    // TEST 2
    std::cout << "Test 2: We now have data in the buffer, time to read it\n";
    uint8_t tempCharStorage; // Location in memory to store the read byte
    // While there is data in the buffer
    while(buffer.getstatus() != 0x00){
        tempCharStorage = buffer.read();
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
        if(buffer.getstatus() != 0xFF){
            buffer.write(data2store2[j]);
            // Increment counter
            j++;
            // Obtain the buffer status and store the last byte as check_byte
            uint8_t check_byte = buffer.peek();
            if ((buffer.getstatus() == 0) && (check_byte == 'l')){
            std::cout << "The letter 'l' has been stored in the buffer\n";
            }

        }else{
            std::cout << "Buffer is Full!\n";
            break;
        }
    }

    return 0;
}


