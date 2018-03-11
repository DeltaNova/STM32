// buffer_class.h
#ifndef BUFFER_CLASS
#define BUFFER_CLASS

#define BUFFER_SIZE 16
#include <stdint.h> // uint8_t

class Buffer {
    public:
        Buffer(){};
        void write(uint8_t byte); // Write data element into buffer.
        void reset();       // Reset the buffer, discarding data.
        uint8_t read();     // Read and return the oldest buffer element value
        uint8_t peek();     // Read and return the last written element value
        // Load an array of a specified size into the buffer.
        uint8_t load(uint8_t *data_array, uint8_t array_size);
        // Updates status based on buffer condition and reutrns status.
        uint8_t getstatus(); // Updates status based on buffer condition and reutrns status.
    private:
        uint8_t data[BUFFER_SIZE];
        // Volatile values as they may get updated by ISR.
        volatile uint8_t newest_index = 0;
        volatile uint8_t oldest_index = 0;
        volatile uint8_t status = 0x00; // 0x00 = Empty, 0x01 = OK, 0xFF = Full

};
#endif //BUFFER_CLASS
