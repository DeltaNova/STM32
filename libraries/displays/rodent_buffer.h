// rodent_buffer.h - SSD1306 OLED Display buffer
// Author: M. Tunstall
/*
  Copyright (C) 2015  Matthew Tunstall

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <stdint.h>         // Enable fixed width integers.

#ifndef RODENT_BUFFER_H
#define RODENT_BUFFER_H

// 128 x 64 Rodent Pattern (Can be optimised, lots of duplicate data)
#ifdef AVR_PROGMEM
    // USAGE: Place '#define AVR_PROGMEM' in main code to store the buffer in 
    //        program memory instead of sram
    #include <avr/pgmspace.h>
    const uint8_t rodent[1024] PROGMEM = {
#endif

#ifndef AVR_PROGMEM
    const uint8_t rodent[1024] = {
#endif
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x10, 0x20, 0x20, 0x30, 0x30, 0x30, 0x30, 0x3C, 0x28, 0x30, 0x20, 0x00, 0x00, // Reversed Rodent
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,

    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,

    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,

    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,

    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,

    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,

    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,

    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x30, 0x28, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00
    };


#endif // End RODENT_BUFFER_H