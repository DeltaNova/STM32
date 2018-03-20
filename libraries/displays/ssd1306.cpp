// ssd1306.cpp - Library for SSD1306 OLED Display Driver
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

#include "ssd1306.h"

void SSD1306::toggleDisplay(bool toggle){
    // Switch Display between On & Sleep Modes
    i2c.start(Address);
    i2c.write(0x00);    // Control Byte Command Stream
    if (toggle){
        i2c.write(0xAF); // Display ON
    }else{
        i2c.write(0xAE); // Display Sleep
    }
    i2c.stop();
}


void SSD1306::toggleDisplayInvert(bool toggle){
    // Switch between normal and Inverted mode.
    i2c.start(Address);
    i2c.write(0x00);    // Control Byte Command Stream
    if (toggle){
        i2c.write(0xA6); // Normal Display
    }else{
        i2c.write(0xA7); // Invert Display
    }
    i2c.stop();
}


void SSD1306::setup(){
    // Initialisation of display based upon SSD1306 Application Note Example
    i2c.start(Address);     // Start Transmitting
    i2c.write(0x00);        // Control Byte Command Stream
    // ---
    i2c.write(0xAE);        // Turn Display Off
    // ---
    i2c.write(0xA8);        // Set Multiplex Ratio
    i2c.write(0x3f);        // 1/64 duty cycle
    // ---
    i2c.write(0xD3);        // Set Display Offset
    i2c.write(0x00);        // No offset applied
    // ---
    i2c.write(0x40);        // Set Display Start Line #0
    // ---
    i2c.write(0xA1);        // Set Segment Remap (Flips Display) A0/A1
    // ---
    i2c.write(0xC8);        // COM Scan Direction c0/c8
    // ---
    i2c.write(0xDA);        // Set COM pins
    i2c.write(0x12);        // 0x12 - See Application Note SW INIT
    // ---
    i2c.write(0x81);        // Set Contrast
    i2c.write(0x7F);        // Default Contrast Level 127/255
    // ---
    i2c.write(0xA4);        // Entire Display On - Output follows RAM Contents
    // ---
    i2c.write(0xA6);        // Set Normal Display 0xA7 inverts
    // ---
    i2c.write(0xD5);        // Display Clk Div
    i2c.write(0x80);        // Default Value 0x80
    // ---
    i2c.write(0x8D);        // Setup Charge Pump - See SSD1306 App Note
    i2c.write(0x14);        // Enable Charge Pump during display on.
    // ---
    i2c.write(0xD9);        // Setup Precharge
    i2c.write(0x22);        // Default Value 0x22
    // ---
    i2c.write(0xDB);        // VCOMH DESELECT
    i2c.write(0x30);        // ~0.83 x Vcc
    // ---
    i2c.write(0x20);        // Set Mem Addr Mode
    i2c.write(0x00);        // Horzontal
    // ---
    i2c.write(0xAF);        // Display On
    // ---
    i2c.stop();             // Stop Transmitting
}


void SSD1306::clear_buffer(){
    // Clear GFX Buffer
    i2c.start(Address);
    i2c.write(0x00);    // Control Byte Command Stream
    i2c.write(0x21);    // Setup Column Addresses
    i2c.write(0x00);    // Col Start Addr
    i2c.write(0x7F);    // Col End Addr
    i2c.write(0x22);    // Set Page Addr
    i2c.write(0x00);    // Start Page 0
    i2c.write(0x07);    // End Page 7
    i2c.stop();

    for (uint16_t i=0; i<1024; i++){
        i2c.start(Address);
        i2c.write(0x40);      // Control Byte Data Stream
        for (uint8_t x=0; x<16; x++) {
            i2c.write(0x00);
            i++;
        }
        i--;
        i2c.stop();
    }
}


void SSD1306::setAddressMode(uint8_t mode){
    // 0 = Horizontal (Default/Fallback)
    // 1 = Vertical
    // 2 = Page
    i2c.start(Address);
    i2c.write(0x00);        // Control Byte Command Stream
    i2c.write(0x20);        // Setup Address Mode
    if (mode == 0){         // Setup Horizontal Mode
        i2c.write(0x00);    // Horizontal
    }
    else if (mode == 1){    // Setup Vertical Mode
        i2c.write(0x01);    // Vertical
    }
    else if (mode == 2){    // Setup Page Mode
        i2c.write(0x02);    // Page
    }
    else{                   // Setup Horizontal Mode as fallback option
        i2c.write(0x00);    // Horizontal
    }
    i2c.stop();
}


void SSD1306::setCursor(uint8_t row_start, uint8_t col_start, uint8_t col_end, uint8_t row_end){
    // Set Cursor Position
    i2c.start(Address);
    i2c.write(0x00);        // Control Byte Command Stream
    i2c.write(0x21);        // Set Column Address
    i2c.write(col_start);   // Start at column 0
    i2c.write(col_end);     // End at column 127
    i2c.write(0x22);        // Set Page (Row) address
    i2c.write(row_start);   // Start Page
    i2c.write(row_end);     // End Page
    i2c.stop();
}


void SSD1306::setColAddr(uint8_t col_start, uint8_t col_end){
    // Set Column Address Range
    i2c.start(Address);
    i2c.write(0x00);        // Control Byte Command Stream
    i2c.write(0x21);        // Set Column Address
    i2c.write(col_start);   // Start at column 0
    i2c.write(col_end);     // End at column 127
    i2c.stop();
}

void SSD1306::setRowAddr(uint8_t row_start, uint8_t row_end){
    // Set Row Address Range
    i2c.start(Address);
    i2c.write(0x00);        // Control Byte Command Stream
    i2c.write(0x22);        // Set Page (Row) address
    i2c.write(row_start);   // Start Page
    i2c.write(row_end);     // End Page
    i2c.stop();
}

void SSD1306::setPageStartCol(uint8_t pscol){
    uint8_t upper_nibble = (((pscol & 0xF0)>>4)|0x10);
    uint8_t lower_nibble = pscol & 0x0F;
    i2c.start(Address);
    i2c.write(0x00); // Control Byte Command Stream
    i2c.write(lower_nibble); // Set Lower Col Start Addr (Page Addr Mode)
    i2c.write(upper_nibble); // Set Upper Col Start Addr (Page Addr Mode)
    i2c.stop();
}

void SSD1306::setPageStartAddress(uint8_t pageStartAddress){
    i2c.start(Address);
    i2c.write(0x00);       // Control Byte Command Stream
    if ((pageStartAddress >= 0x00) && (pageStartAddress <= 0x07)){
        i2c.write((pageStartAddress | 0xB0)); // Write B0 - B7, Page 0 - Page 7
    }else{
        i2c.write(0xB0); // Default to Page 0
    }
    i2c.stop();
}

void SSD1306::setupHorizScroll(uint8_t direction, uint8_t startPage, uint8_t endPage, uint8_t interval){
    // Continuous Horizontal Scroller

    i2c.start(Address);
    i2c.write(0x00);    // Control Byte Command Stream
    if (direction == 0x00){
        i2c.write(0x26); // Scroll Right
    }else{
        i2c.write(0x27); // Scroll Left
    }
    i2c.write(0x00);        // Dummy Byte 0x00
    i2c.write(startPage);   // Start Page Address
    i2c.write(interval);    // Time interval between scroll steps
    i2c.write(endPage);     // End Page Address
    i2c.write(0x00);        // Dummy Byte 0x00
    i2c.write(0xFF);        // Dummy Byte 0xFF
    i2c.stop();
}

void SSD1306::setupHVScroll(uint8_t direction, uint8_t startPage, uint8_t endPage, uint8_t interval, uint8_t voffset){
    // Horizontal & Vertical Scroller

    i2c.start(Address);
    i2c.write(0x00);    // Control Byte Command Stream
    if (direction == 0x00){
        i2c.write(0x29); // Scroll Right
    }else{
        i2c.write(0x2A); // Scroll Left
    }
    i2c.write(0x00);        // Dummy Byte 0x00
    i2c.write(startPage);   // Start Page Address
    i2c.write(interval);    // Time interval between scroll steps
    i2c.write(endPage);     // End Page Address
    i2c.write(voffset);        // Vertical Offset
    i2c.stop();
}

void SSD1306::setupVertScrollArea(uint8_t fixedTopRows, uint8_t scrollRows){
    // Setup Vertical Scrolling Area
    i2c.start(Address);
    i2c.write(0x00);    // Control Byte Command Stream
    i2c.write(0xA3);    // Set Vertical Scroll Area
    i2c.write(fixedTopRows); // Number of fixed rows at top of display.
    i2c.write(scrollRows); // Number of rows to scroll below fixed rows.
    i2c.stop();
}

void SSD1306::scrollToggle(bool toggle){
    // Toggle Scrolling
    i2c.start(Address);
    i2c.write(0x00);    // Control Byte Command Stream
    if (toggle){
        i2c.write(0x2F); // TRUE - Activate Scroll
    }else{
        i2c.write(0x2E); // FALSE - Deactivate Scroll
    }
    i2c.stop();
}

void SSD1306::drawBuffer(const uint8_t *buffer_to_draw){
    // Draw buffer on display
    i2c.start(Address);
    i2c.write(0x00);    // Control Byte Command Stream
    i2c.write(0x21);    // Setup Column Addresses
    i2c.write(0x00);    // Col Start Addr
    i2c.write(0x7F);    // Col End Addr
    i2c.write(0x22);    // Set Page Addr
    i2c.write(0x00);    // Start Page 0
    i2c.write(0x07);    // End Page 7
    i2c.stop();

    for (uint16_t i=0; i<1024; i++){
        i2c.start(Address);
        i2c.write(0x40);      // Control Byte Data Stream
        for (uint8_t x=0; x<16; x++) {
            i2c.write(buffer_to_draw[i]);
            i++;
        }
        i--;
        i2c.stop();
    }
}

void SSD1306::write(uint8_t char2write, const uint8_t* symbol_buffer){
    // Write Character, symbol buffer used from PROGMEM
    // Note: Set cursor position before writing.
    i2c.start(Address);
    i2c.write(0x40);                   // Control Byte Data Stream
    // Multiply by 6 to get char start position in the ascii_buffer
    uint16_t offset =  char2write * 6;
    // Print each byte that makes up the character
    for ( uint16_t x =0; x<6; x++) {
        i2c.write(symbol_buffer[offset+x]);
    }
    i2c.stop();

    // Note: Reset the cursor position to Row 0, Col 0. Otherwise the
    //       buffers become offset on the next loop.
}

////////////////////////////////////////////////////////////////////////////////
// AVR Specific Functions - Using PROGMEM
#ifdef AVR_PROGMEM
    // USAGE: Place '#define AVR_PROGMEM' in main code to store the buffer in 
    //        program memory instead of sram

void SSD1306::PROGMEMwriteBuf(const uint8_t* buffer_to_write){
    for (uint16_t i=0; i<1024; i++){
        i2c.start(Address);
        i2c.write(0x40);      // Control Byte Data Stream
        for (uint8_t x=0; x<16; x++) {
            i2c.write(pgm_read_byte(&buffer_to_write[i]));
            i++;
        }
        i--;
        i2c.stop();
    }
}

void SSD1306::PROGMEMwriteLine(const uint8_t* buffer_name, uint8_t buffer_length, const uint8_t* symbol_buffer){
    // Write Text Display Line
    // Note: Set cursor position before writing.

    // For each item in data buffer
    for (uint16_t i=0; i<buffer_length; i++){
        i2c.start(Address);
        i2c.write(0x40);                   // Control Byte Data Stream
        // Multiply by 6 to get char start position in the ascii_buffer
        uint16_t offset = pgm_read_byte(&buffer_name[i]) * 6;
        // Print each byte that makes up the character
        for ( uint16_t x =0; x<6; x++) {
            i2c.write(pgm_read_byte(&symbol_buffer[offset+x]));
        }
        i2c.stop();
    }
    // Note: Reset the cursor position to Row 0, Col 0. Otherwise the
    //       buffers become offset on the next loop.
}


void SSD1306::PROGMEMwriteLine2(uint8_t* buffer_name, uint8_t buffer_length, const uint8_t* symbol_buffer){
    // Write Text Display Line, symbol_buffer used from PROGMEM
    // Note: Set cursor position before writing.

    // For each item in data buffer
    for (uint16_t i=0; i<buffer_length; i++){
        i2c.start(Address);
        i2c.write(0x40);                   // Control Byte Data Stream
        // Multiply by 6 to get char start position in the ascii_buffer
        uint16_t offset = buffer_name[i] * 6;
        // Print each byte that makes up the character
        for ( uint16_t x =0; x<6; x++) {
            i2c.write(pgm_read_byte(&symbol_buffer[offset+x]));
        }
        i2c.stop();
    }
    // Note: Reset the cursor position to Row 0, Col 0. Otherwise the
    //       buffers become offset on the next loop.
}


void SSD1306::PROGMEMwriteChar(uint8_t char2write, const uint8_t* symbol_buffer){
    // Write Character, symbol buffer used from PROGMEM
    // Note: Set cursor position before writing.
    i2c.start(Address);
    i2c.write(0x40);                   // Control Byte Data Stream
    // Multiply by 6 to get char start position in the ascii_buffer
    uint16_t offset =  char2write * 6;
    // Print each byte that makes up the character
    for ( uint16_t x =0; x<6; x++) {
        i2c.write(pgm_read_byte(&symbol_buffer[offset+x]));
    }
    i2c.stop();

    // Note: Reset the cursor position to Row 0, Col 0. Otherwise the
    //       buffers become offset on the next loop.
}
#endif // AVR_PROGMEM
