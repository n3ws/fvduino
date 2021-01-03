
/* Based on:
   
   ATtiny85 Colour Graphics Library v2 - see http://www.technoblogy.com/show?2EA7

   David Johnson-Davies - www.technoblogy.com - 17th December 2018
   ATtiny85 @ 8 MHz (internal oscillator; BOD disabled)
   
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
*/


#include "oled.h"
#include "font.h"

// These are bit numbers in PORTB
const uint8_t data = 5; //Pin 13;
const uint8_t cs = 4;   //Pin 12;
const uint8_t clk = 3;  //Pin 11;

// OLED 96x64 colour display **********************************************

// Initialisation sequence for OLED module
const int InitLen = 25;
const unsigned char Init[InitLen] PROGMEM = {
  0xAE,         // Display off 
  0xA0,         // Driver remap and colour depth
  //0x22,       // COM split, flip horizontal:     0010 0010
  0b00110000,   // COM splite, rotate 180 degrees: 0011 0000
                // bit 0 - address increment mode
                // bit 1 - column address mapping (mirror left - right)
                // bit 2 - RGB mapping
                // bit 3 - COM left/right remap
                // bit 4 - COM scan direction remap (mirror up - down)
                // bit 5 - Odd Even Split of COM pins
                // bits 6 and 7 - Display color mode
  0xA8, 0x3F,   // Multiplex
  0xAD, 0x8E,   // External supply
  0x8A, 0x64,   // Set Second Pre-charge Speed of Color A
  0x8B, 0x78,   // Set Second Pre-charge Speed of Color B
  0x8C, 0x64,   // Set Second Pre-charge Speed of Color C
  0xBB, 0x3A,   // Set Pre-charge Level
  0xBE, 0x3E,   // Set VCOMH
  0x87, 0x0D,   // Set Master Current Control. Values 00 - 0F, default 06
  0x81, 0x91,   // Set Contrast Control for Color A
  0x82, 0x50,   // Set Contrast Control for Color B
  0x83, 0x7D,   // Set Contrast Control for Color C
};

// Send a byte to the display
void Send(uint8_t d) 
{  
  for (uint8_t bit = 0x80; bit; bit >>= 1) {
    PINB = 1<<clk;                        // clk low
    if (d & bit) PORTB = PORTB | (1<<data); else PORTB = PORTB & ~(1<<data);
    PINB = 1<<clk;                        // clk high
  }
}

void init_display() 
{    
  DDRB = 1<<clk | 1<<cs | 1<<data;    // All outputs
  PORTB = 1<<clk | 1<<cs;             // clk and cs high
  
  PINB = 1<<cs;                           // cs low
  for (uint8_t c=0; c<InitLen; c++) Send(pgm_read_byte(&Init[c]));
  PINB = 1<<cs;                           // cs high
}

// Display off = 0, on = 1
void display_on(uint8_t on) 
{
  PINB = 1<<cs;                           // cs low
  Send(0xAE + on);
  PINB = 1<<cs;                           // cs high
}

// Graphics **********************************************

// Clear display
void clear_display() 
{
  PINB = 1<<cs;                           // cs low
  Send(0x25);                             // Clear Window
  Send(0); Send(0); Send(95); Send(63);
  PINB = 1<<cs;                           // cs high
  //delayMicroseconds(900);
  delayMicroseconds(450);                 // half delay when clocking nano to 8 MHz
}

// Draw line to (x,y) in given colour
void draw_line (uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t col) 
{  
  uint8_t red   = (col & 0xf800) >> 10;
  uint8_t green = (col & 0x07e0) >> 5;
  uint8_t blue  = (col & 0x001f) << 1;
  
  PINB = 1<<cs;                           // cs low
  Send(0x21);                             // Draw Line
  Send(x0); Send(y0); Send(x1); Send(y1);
  Send(red); Send(green); Send(blue);
  PINB = 1<<cs;                           // cs high
}

// Plot a point at (x,y)
void plot_point (uint8_t x, uint8_t y, uint16_t col) 
{
  draw_line(x, y, x, y, col);
}

// Draw a rectangle in foreground colour optionally filled another colour
void draw_rect (uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
               uint16_t edgecol, uint16_t fillcol, boolean filled) 
{  
  if(w == 0 || h == 0)
    return;

  uint8_t e_red   = (edgecol & 0xf800) >> 10;
  uint8_t e_green = (edgecol & 0x07e0) >> 5;
  uint8_t e_blue  = (edgecol & 0x001f) << 1;

  uint8_t f_red   = (fillcol & 0xf800) >> 10;
  uint8_t f_green = (fillcol & 0x07e0) >> 5;
  uint8_t f_blue  = (fillcol & 0x001f) << 1;
 
  
  PINB = 1<<cs;                           // cs low
  Send(0x26); Send(filled);               // Enable fill
  Send(0x22);                             // Draw rectangle
  Send(x); Send(y); Send(x + w - 1); Send(y + h - 1);
  Send(e_red); Send(e_green); Send(e_blue);
  Send(f_red); Send(f_green); Send(f_blue);
  PINB = 1<<cs;                           // cs high

  if(filled)                              // filled rectangles take time to draw
    //delayMicroseconds(200);               // must not issue new command too soon
    delayMicroseconds(100);               // half delay when clocking nano to 8 MHz
}

// Plot character in given colour
void plot_char (uint8_t x, uint8_t y, uint8_t ch, uint16_t col, uint8_t scale) 
{
  uint8_t red   = (col & 0xf800) >> 10;
  uint8_t green = (col & 0x07e0) >> 5;
  uint8_t blue  = (col & 0x001f) << 1;
  
  PINB = 1<<cs;                           // cs low
  for (uint8_t c = 0 ; c < 5; c++) {      // Column range
    uint8_t bits = pgm_read_byte(&font[ch - 32][c]);
    uint8_t r = 0;
    while (bits) {
      while ((bits & 1) == 0) {r++; bits = bits>>1; }
      uint8_t on = (7 - r) * scale;
      while ((bits & 1) != 0) {r++; bits = bits>>1; }
      uint8_t off = (7 - r) * scale + 1;
      for (int i=0; i < scale; i++) {
        uint8_t h = x + c * scale + i;
        Send(0x21);                         // Draw line
        Send(h); Send(y + on); Send(h); Send(y + off);
        Send(red); Send(green); Send(blue);
      }
    }
  }
  PINB = 1<<cs;                             // cs high
}

// Plot text
void plot_text(uint8_t x, uint8_t y, const char* text, uint16_t col, uint8_t scale) 
{
  while (1) {
    char c = *(text++);
    if (c == 0) return;
    plot_char(x, y, c, col, scale);
    x += 6;
  }
}

// Higher level functions for drawing ui elements

// convert unsigned byte to char array
// *s points to end() of buffer
char *btoa(uint8_t x, char *s)
{
    *--s = 0;
    if (!x) *--s = '0';
    for (; x; x /= 10) 
      *--s = '0' + x % 10;
    return s;
}


// TODO: Change ui to work as Master volume and mix
// Master volume: -10 db - +4 db
// Mix: 0 - 50% - 100% (Not all values are possible, map to pot values)
// New ui elements for vol and mix?
// Valid values for dir are -1 and 1


// Draw volume -15 ------ 0 --- +4db / Mute
// Values given are 0-20.
void draw_volume(const uint16_t* palette, const uint16_t bg, const uint8_t pos, const uint8_t val)
{
  uint8_t start = pos * 9 + 1; 
  uint8_t len = map(val, 0, 20, 0, 94);
  uint16_t barcol  = palette[0];
  uint16_t textcol = palette[3];
 
  draw_rect(1 , start, 94, 8, bg, bg);            // Erase old bar
  draw_rect(1 , start, len, 8, barcol, barcol);   // Draw new bar

  plot_text(2, start, "Vol", textcol);

  // The code for constructing these on the fly 
  // takes as much space, but the array is simpler.
  static const char db[21][4] = {"-80",
                                 "-16", "-15", "-14", "-13", 
                                 "-12", "-11", "-10", " -9", 
                                 " -8", " -7", " -6", " -5", 
                                 " -4", " -3", " -2", " -1", 
                                 " +0", " +1", " +2", " +3"};
  
  plot_text(31, start, db[val], textcol); 
  plot_text(49, start, "db", textcol);
}

// Draw mix as Dry ---  50% ---- Wet
// Values given are 0-20.
void draw_mix(const uint16_t* palette, const uint16_t bg, const uint8_t pos, const uint8_t val)
{
  uint8_t start = pos * 9 + 1; 
  uint8_t len = map(val, 0, 20, 0, 94);
  uint16_t barcol  = palette[0];
  uint16_t textcol = palette[3];

  draw_rect(1 , start, 94, 8, bg, bg);            // Erase old bar
  draw_rect(1 , start, len, 8, barcol, barcol);   // Draw new bar

  plot_text(2, start, "Dry", textcol);
  plot_text(77, start, "Wet", textcol);

  char buff[4];                  // 3 chars + \0
  buff[0] = buff[1] = ' ';       // put spaces in tens and hundreds position
  
  btoa(val * 5, buff + 4); // give pointer to just after buffer (end)

  plot_text(37, start, buff, textcol); 
}


//  Draws bar with name and value
 // Parameter values shown as 0-100%
void draw_parameter(const uint16_t* palette, const uint16_t bg, const char* text, const uint8_t pos, const uint8_t val)
{
  uint8_t start = pos * 9 + 1; 
  uint8_t len = map(val, 0, 255, 0, 94);
  uint8_t perc = map(val, 0, 255, 0, 100);
  uint16_t barcol  = palette[0];
  uint16_t textcol = palette[3];

  draw_rect(1 , start, 94, 8, bg, bg);            // Erase old bar
  draw_rect(1 , start, len, 8, barcol, barcol);   // Draw new bar

  plot_text(2, start, text, textcol);

  char buff[4];                  // 3 chars + \0 // add percent mark?
  buff[0] = buff[1] = ' ';       // put spaces in tens and hundreds position
  
  btoa(perc, buff + 4); // give pointer to just after buffer (end)

  // we want right justified, give buff instead of res
  plot_text(77, start, buff, textcol); 
}
