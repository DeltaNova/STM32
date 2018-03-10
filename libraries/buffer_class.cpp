// buffer_class.cpp - Circular Buffer Class
#include "buffer_class.h"

void Buffer::write(uint8_t byte){
    // Write data into next available buffer location.
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
    uint8_t next_index = ((newest_index + 1)% BUFFER_SIZE);
    // Is Buffer full?
    if (next_index == oldest_index){
        status = 0xFF; // Buffer Full   
    }else{
        status = 0x01; // Buffer OK, not empty as we are adding data.
    }

    data[newest_index] = byte;
    newest_index = next_index;
}

uint8_t Buffer::read(){
    // Zero read_data, allows us to return zero if buffer is empty.
    uint8_t read_data = 0; 
    
    if (newest_index == oldest_index){
        status = 0x00; // Buffer Empty
        return read_data;
    }
    
    // Read oldest data in buffer
    read_data = data[oldest_index]; 
    
    // Update the the oldest_index to the oldest unread data.
    oldest_index = ((oldest_index + 1) % BUFFER_SIZE); 
    
    // Update buffer status. 
    // TODO: Buffer could be full if data added to it via an interrupt.
    
    if (newest_index == oldest_index){
        status = 0x00; // Buffer Empty
    }else{
        status = 0x01; // Buffer OK
    }
    return read_data;
    
}

uint8_t Buffer::peek(){
    /*
    https://hackaday.com/2015/10/29/embed-with-elliot-going-round-with-circular-buffers/
    "One trick is in defining the last_index. When the buffer has just wrapped
    around, newest_index can be equal to zero. Subtracting one from that (to go
    back to the last written character) will end up with a negative number in
    that case. Adding another full BUFFER_SIZE to the index prevents the value
    from ever going negative, and the extra factor of BUFFER_SIZE gets knocked
    out by the modulo operation anyway."
    */
    // peek looks inside the buffer at the last byte stored and returns it.
    uint8_t last_index = ((BUFFER_SIZE + (newest_index - 1)) % BUFFER_SIZE);
    if (newest_index == oldest_index){
        status = 0x00; // Buffer Empty
    }
    
    uint8_t last_byte = data[last_index];
    status = 0x01; // Buffer OK // TODO: Could also be full.
    return last_byte;
}
    
uint8_t Buffer::load(uint8_t *data_array, uint8_t array_size){
    // Loads an array in the buffer
    // Load will continue until complete or until buffer becomes full.
    
    // DEV NOTE:
    // C++ does not copy an array when an array is passed into a function. When
    // passing an array as an argument to a function, a fixed array decays into
    // a pointer, and the pointer is passed to the function.
    // Array size needs to be passed as an additional variable.
    
    uint8_t i = 0;                  // Array Index
    // While there is data to store and room in the buffer
    while((i < (array_size)) && (getstatus() != 0xFF)){
        write(data_array[i]);   // Write array element into buffer
        i++;                    // Increment array index.
    }
    
    // Return 0 if all data loaded. None zero value is total unloaded elements.
    return (array_size - i);
}

uint8_t Buffer::getstatus(){
    // Update the status of the buffer and return it.
    
    // Calculate the next location inthe buffer.
    uint8_t next_index = (((newest_index)+1) % BUFFER_SIZE);
    // If the next location contains the oldest data then we have a full buffer.
    if (next_index == oldest_index){
        status = 0xFF; // Buffer Full
    } else if (newest_index == oldest_index){ // No data in the buffer.
        status = 0x00; // Buffer Empty
    } else { // Buffer contains some data but has space for more.
        status = 0x01; // Buffer OK
    }
    return status;
}

