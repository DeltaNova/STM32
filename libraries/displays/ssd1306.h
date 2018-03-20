// ssd1306.h - Header file for ssd1306.cpp
// Library for SSD1306 OLED Display Driver
// Author: M. Tunstall
/*
  Copyright (C) 2015, 2018  Matthew Tunstall

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
#include "i2c.h"            // Access to the I2C Class
#ifndef SSD1306_H 
#define SSD1306_H

class SSD1306
{
    protected: // accessed by member functions and derived classes.
        I2C& i2c; // Reference to an instance of the I2C interface.
    private: // accessed by member functions but not derived classes.
        uint8_t Address;
    public: // accessed by anybody.
        SSD1306(I2C& i2c, uint8_t addr)     // Default Constructor
        :i2c(i2c),Address(addr){}   
        ~SSD1306(){}                        // Destructor
        void setup();
        void clear_buffer();
        void setAddressMode(uint8_t mode);
        void setCursor(uint8_t row_start = 0x00, uint8_t col_start = 0x00, uint8_t col_end = 0x7F, uint8_t row_end = 0x07);
        void setColAddr(uint8_t col_start = 0x00, uint8_t col_end = 0x7F);
        void setRowAddr(uint8_t row_start = 0x00, uint8_t row_end = 0x07);
        void setPageStartCol(uint8_t pscol = 0x00);
        void setPageStartAddress(uint8_t pageStartAddress = 0x00);
        

        // Toggle Scrolling - OFF: FALSE, ON: TRUE
        void scrollToggle(bool toggle);
        // Continuous Horizontal Scroller
        void setupHorizScroll(uint8_t direction, uint8_t startPage, uint8_t endPage, uint8_t interval);
        // Horizontal & Vertical Scroll
        void setupHVScroll(uint8_t direction, uint8_t startPage, uint8_t endPage, uint8_t interval, uint8_t voffset);
        // Vertical Scroll Area
        void setupVertScrollArea(uint8_t fixedTopRows, uint8_t scrollRows);
        void toggleDisplay(bool toggle=1);
        void toggleDisplayInvert(bool toggle = 1);
        
        
        // DEVNOTE: C++ does not allow an entire array to a function. 
        //          buffer_to_draw is therefore accessed using a pointer. 
        void drawBuffer(const uint8_t *buffer_to_draw);
        
        void writeLine(uint8_t &line_buffer, uint8_t buffer_length, uint8_t *symbol_buffer); // TODO: Need to implement
        void write(uint8_t char2write, const uint8_t *symbol_buffer);

        ////////////////////////////////////////////////////////////////////////
        // AVR Specific Functions
        //----------------------------------------------------------------------
        #ifdef AVR_PROGMEM
        // USAGE: Place '#define AVR_PROGMEM' in main code to store the buffer in 
        //        program memory instead of sram
        #include <avr/pgmspace.h>
        
        // PROGMEMwriteBuf - Use to display contents of buffer stored in PROGMEM
        void PROGMEMwriteBuf(const uint8_t* buffer_to_write);
        
        // PROGMEMwriteLine2 - Use to display line from buffer.
        //                   - Use when buffer "IS NOT" stored in PROGMEM
        //                   - Use when symbol_buffer "IS" stored in PROGMEM
        void PROGMEMwriteLine2(uint8_t* buffer_name, uint8_t buffer_length, const uint8_t* symbol_buffer);
        
        // PROGMEMwriteLine - Use to display line from buffer.
        //                  - Use when buffer "IS" stored in PROGMEM
        //                  - Use when symbol_buffer "IS" stored in PROGMEM
        void PROGMEMwriteLine(const uint8_t* buffer_name, uint8_t buffer_length, const uint8_t* symbol_buffer);
        
        // PROGMEMwriteChar - Write a single character to screen
        //                  - Use when symbol buffer "IS" stored in PROGMEM
        void PROGMEMwriteChar(uint8_t char2write, const uint8_t* symbol_buffer);
        #endif //AVR_PROGMEM
};

#endif // End of ifndef SSD1306_H
