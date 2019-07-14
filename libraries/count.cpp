// count.cpp
#include "count.h"

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
