/* Based on:
   
   ATtiny85 Colour Graphics Library v2 - see http://www.technoblogy.com/show?2EA7

   David Johnson-Davies - www.technoblogy.com - 17th December 2018
   ATtiny85 @ 8 MHz (internal oscillator; BOD disabled)
   
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
*/

#include "Arduino.h"

#ifndef oled_h
#define oled_h

// Other color definitions
const uint16_t BLACK  = 0x0000;
const uint16_t MARKED = 0x3000; //0x0020
const uint16_t NONE   = 0xFFFF; // this is used for rectangle inner color when not used

void init_display();

void display_on(uint8_t on);

void clear_display();

void draw_line (uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t col);

void plot_point (uint8_t x, uint8_t y, uint16_t col);

void draw_rect (uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
               uint16_t edgecol, uint16_t fillcol, boolean filled = true);

void plot_char (uint8_t x, uint8_t y, uint8_t ch, uint16_t col, uint8_t scale = 1);

void plot_text(uint8_t x, uint8_t y, const char* text, uint16_t col, uint8_t scale = 1);

char *btoa(uint8_t x, char *s);

void draw_volume(const uint16_t* palette, const uint16_t bg, const uint8_t pos, const uint8_t val);

void draw_mix(const uint16_t* palette, const uint16_t bg, const uint8_t pos, const uint8_t val);

void draw_parameter (const uint16_t* palette, const uint16_t bg, const char* text, const uint8_t pos, const uint8_t val);

#endif
