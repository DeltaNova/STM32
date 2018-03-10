// buffer_class.h

#define BUFFER_SIZE 16
#include <stdint.h> // uint8_t

class Buffer {
    public:
        Buffer(){};
        void write(uint8_t byte);
        uint8_t read();
        uint8_t peek();
        uint8_t load(uint8_t *data_array, uint8_t array_size);
        uint8_t getstatus(); // Updates status based on buffer condition and reutrns status.
    private:
        uint8_t data[BUFFER_SIZE];
        // Volatile values as they may get updated by ISR.
        volatile uint8_t newest_index = 0;
        volatile uint8_t oldest_index = 0;
        volatile uint8_t status = 0x00; // 0x00 = Empty, 0x01 = OK, 0xFF = Full

};
