// count.h
#ifndef COUNT_H
#define COUNT_H

#include <stdint.h>             // Fixed Width Integers


uint16_t getCountDiff(uint16_t count, uint16_t last_count);
uint32_t get_upcounting_delta(uint32_t start_count, uint32_t stop_count);

#endif
