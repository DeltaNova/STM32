// count.cpp
#include "count.h"


uint16_t getCountDiff(uint16_t count, uint16_t last_count){
    // This function assumes a count range of 0x0000 to 0xFFFF with roll over
    // and roll under.
    
    // Note: It will not work correctly if the count is scaled before use.
    
    // A window is used to assess the difference between the count and 
    // last_count values. Values inside or outside of the window determine how 
    // the diff is calculated. Multiple rollover not handled.
    uint16_t diff;
    if (count > last_count){
        if (count < 0x8000){
            return (count - last_count);            // Increment
        }else{ // (count >= 0x8000)
            if (last_count > (count-0x4000)){
                return (count - last_count);        // Increment
            }else{ // (last_count <= (count-0x4000))
                // Count Roll Under Condition
                diff = (0xFFFF - count);
                return(last_count + diff);          // Decrement
            }
        }
    }else{ // (count <= last count)
        if (last_count < 0x8000){
            return(last_count - count);             // Decrement
        }else{ // (last_count >= 0x8000)
            if (count > (last_count - 0x4000)){
                return(last_count - count);         // Decrement
            }else{ //(count <= (last_count - 0x4000))
                // Count Roll Over Condition
                diff = 0xFFFF - last_count;
                return (count + diff);              // Increment
            }
        }
    }
}


uint32_t get_upcounting_delta(uint32_t start_count, uint32_t stop_count){
    // Returns the difference between two count values.
    
    // DEV NOTE: Count values need to be be from an upcounting only counter.
    //           Decrementing counts can result in large deltas where only a
    //           small delta exits in reality.
    
    if (stop_count < start_count){
        // Counter has rolled over (at least once, multi rollover not handled)
        return ((0xFFFFFFFF - start_count) + 1 + stop_count);
    }else{
        return (stop_count - start_count);
    }
}
