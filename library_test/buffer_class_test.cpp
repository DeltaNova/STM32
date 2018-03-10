// buffer_test.cpp - Circular Buffer Example testing buffer.h/buffer.cpp
#include "../libraries/buffer_class.h"
#include <stdint.h> // uint8_t
#include <iostream> // std::cout
#include <bitset> // Used to output values to cout
/*
    COMPILE
    -------
    g++ -std=gnu++11 -L buffer_class -W -o buffer_class_test buffer_class_test.cpp buffer_class.cpp
*/

int main(void){
    std::cout << "This is a test of a circular buffer.\n";
    std::cout << " ** Note: Buffer can hold 15 entries **\n";
    /* Begin by defining an instance of the Buffer called buffer.
     * BUFFER_SIZE is defined as 16 in the header. Actual number of storage 
     * elements is equal to (BUFFER_SIZE - 1).
    */
    Buffer buffer;

    ////////////////////////////////////////////////////////////////////////////
    // TEST 1 - Putting data into the buffer.
    // -------------------------------------------------------------------------
    std::cout << "\nTest 1: We will start by storing a message in the buffer\n";
    std::cout << " ** Buffer is reset between tests **\n\n";
    // The data exeeds the buffer size so not all of it can be stored.
    // Define data2store using C-string. 26 characters are listed but sizeof()
    // will return 27 since the string is null terminated (not shown).
    std::cout << " - Test 1a - Store oversized message, 27 entries\n";
    uint8_t test1a [] = "abcdefghijklmnopqrstuvwxyz";
    uint8_t load_status;
      
    // Put data into the buffer & return status of load process.
    load_status = buffer.load(test1a, sizeof(test1a));
    if (load_status != 0x00){ // If buffer unable to write all data.
        std::cout << "  - Buffer Full: ";
        std::cout << int(load_status); // Convert to printable integer.
        std::cout << " items could not be stored.\n" ;
        std::cout << " \033[1;32m[Pass]\033[0m\n"; // PASS
    }else{
        std::cout << " \033[1;31m[Fail]\033[0m\n"; // FAIL
    }
    std::cout << "\n - Test 1b - Store message of 15 entries\n";
    buffer.reset(); // Start with empty buffer.
    uint8_t test1b [] = "Hello World!!!"; // 14 Entries + hidden null terminator.
    // Put data into the buffer & return status of load process.
    load_status = buffer.load(test1b, sizeof(test1b));
    if (load_status != 0x00){ // If buffer unable to write all data.
        std::cout << "  - Buffer Full: ";
        std::cout << int(load_status); // Convert to printable integer.
        std::cout << " items could not be stored.\n" ;
        std::cout << " \033[1;31m[Fail]\033[0m\n"; // FAIL
    }else{
        std::cout << " \033[1;32m[Pass]\033[0m\n"; // PASS
    }
    
    std::cout << "\n - Test 1c - Store small message of entries\n";
    buffer.reset(); // Start with empty buffer.
    uint8_t test1c [] = "Hello!"; // 6 Entries + hidden null terminator.
    // Put data into the buffer & return status of load process.
    load_status = buffer.load(test1c, sizeof(test1c));
    if (load_status != 0x00){ // If buffer unable to write all data.
        std::cout << "  - Buffer Full: ";
        std::cout << int(load_status); // Convert to printable integer.
        std::cout << " items could not be stored.\n" ;
        std::cout << " \033[1;31m[Fail]\033[0m\n"; // FAIL
    }else{
        std::cout << " \033[1;32m[Pass]\033[0m\n"; // PASS

        
    }
    ////////////////////////////////////////////////////////////////////////////
    // TEST 2
    std::cout << "\nTest 2: We now have data in the buffer, time to read it\n\n";
    
    uint8_t tempCharStorage; // Location in memory to store the read byte
    std::cout << "The buffer is loaded with: Hello!\n";
    std::cout << "Contents read from buffer: "; 

    // While there is data in the buffer
    while(buffer.getstatus() != 0x00){
        tempCharStorage = buffer.read();
        std::cout << tempCharStorage;
    }
    
    
    // The buffer is now empty.

    ////////////////////////////////////////////////////////////////////////////
    // TEST 3
    std::cout << "\n\nTest 3: Try out bufferPeek()\n";
    buffer.reset(); // Reset buffer for new test.
    uint8_t check_byte; // Storage for output of bufferPeek()
    // Lets add some data to the buffer.

    // This is the data to put into the buffer.
    uint8_t test3 [] = "Hello!\n";
     std::cout << "The buffer will be loaded with: Hello!\n";
     std::cout << "We will peek into the buffer after each letter is added. ";
     std::cout << "The program will report when the letter 'l' is stored.\n\n";
    // Zero Counter
    uint8_t j = 0;

    // While there continues to be data to store in the buffer.
    while(j < sizeof(test3) - 1){
        // Store data byte in buffer unless it is full
        if(buffer.getstatus() != 0xFF){
            buffer.write(test3[j]);
            // Increment counter
            j++;
            // Obtain the buffer status and store the last byte as check_byte
            uint8_t check_byte = buffer.peek();
            
            if (check_byte == 'l'){
            std::cout << "The letter 'l' has been stored in the buffer\n";
            }

        }else{
            std::cout << "Buffer is Full!\n";
            break;
        }
    }

    return 0;
}


