
/*  Copyright 2020 Perttu Haimi
  
    This file is part of FVduino.

    FVduino is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FVduino is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FVduino.  If not, see <https://www.gnu.org/licenses/>.
 * 
 * 
 */
 
#include <Arduino.h>
#include <Bounce2.h>
#include <EEPROM.h>

//#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>

#include  <avr/power.h>

typedef unsigned char PROGMEM prog_uchar;

#include "HW_I2CMaster.h"
#include "oled.h"

// Pin configuration

// Digital potentiometer uses hardware I2C (TWI), pins A4 and A5
const uint8_t POT_ENABLE_PIN = A3;
//const uint8_t POT_SDA_PIN = A4;
//const uint8_t POT_SCL_PIN = A5;

// Three PWM outputs
const uint8_t PAR1_PWM_PIN = 3;
const uint8_t PAR2_PWM_PIN = 9;
const uint8_t PAR3_PWM_PIN = 10;

// Three buttons.
const uint8_t FOOT_BUT_PIN = 8;
const uint8_t SAVE_BUT_PIN = 7;
const uint8_t SELE_BUT_PIN = 2;

// Encoder with button
const uint8_t ENC1_PIN = A0;
const uint8_t ENC2_PIN = A1;
const uint8_t FAST_BUT_PIN = A2;

// EEPROM emulation and FV-1 program switching
const uint8_t PC_NOTIFY_PIN = 4;
const uint8_t I2C_EEPROM_EMUL_SDA_PIN = 5;
const uint8_t I2C_EEPROM_EMUL_SCL_PIN = 6;
const uint8_t EEPROM_EMUL_ADDR = 0x50;

// OLED display commununication
// For reference here, oled code uses port and bit banging
//const uint8_t OLED_DATA_PIN = 13;
//const uint8_t OLED_CS_PIN   = 12;
//const uint8_t OLED_CLK_PIN  = 11;

// Digipot address
const uint8_t DS1881_BASE_I2C_ADDR = 0x28;
const uint8_t DS1881_write_address = DS1881_BASE_I2C_ADDR << 1;
const uint8_t DS1881_read_address = (DS1881_BASE_I2C_ADDR << 1) | _BV(0);


/*********** FV-1 algorithm data structures ***********/

// The eeprom data is in 16 * 32 byte banks (total 512 bytes)
// They are compressed by LZ algorithm (compress.py) to approx.
// half the size. Include the algorithm data blocks here.

#include "spinsemi.h"

// Strings for describing the algorithm and parameters
typedef struct {
  char algoname [14];  
  char par1name [6];
  char par2name [6];
  char par3name [6];
} Description;

// Sensible default settings for each algorithm
typedef struct {
  uint8_t vol;
  uint8_t mix;
  uint8_t par1;
  uint8_t par2;
  uint8_t par3;
} Default;

// Keep everything together in a single array
typedef struct {
  Description desc;
  Default def;
  uint8_t* const prog_addr;
} AlgoDatum;

const AlgoDatum algodata [] PROGMEM = {

  // Reverbs

  {{"SprgRev+Trem",  "Revrb", "Rate", "Depth"}, {17,  8, 128, 128, 128}, (uint8_t *)spring_verb},
  {{"Tremolo +Rev", "Revrb", "Rate",  "Level"}, {17,  8, 128, 128, 128}, (uint8_t *)GA_DEMO_TREM},
 
  {{"Room",          "Pre",   "Time",  "Damp" }, {17,  8, 128, 128, 128}, (uint8_t *)K3_V1_1_Room},
  {{"Hall",          "Pre",   "Time",  "Damp" }, {17,  8, 128, 128, 128}, (uint8_t *)K3_V1_0_Hall},
  {{"Gated Reverb",  "Pre",   "GateT", "Damp" }, {17,  8, 128, 128, 128}, (uint8_t *)K3_V1_3_GATED},

  {{"Reverb+HP+LP",  "Revrb", "HP",    "LP"   }, {17, 10, 128, 128, 128}, (uint8_t *)dance_ir_h_l},
  {{"Rev+Pitch+LP",  "Revrb", "Pitch", "LP"   }, {17, 10, 128, 128, 128}, (uint8_t *)dance_ir_ptz_l},
  {{"Rev+Flang+LP",  "Revrb", "Flang", "LP"   }, {17, 10, 128, 128, 128}, (uint8_t *)dance_ir_fla_l},

  {{"ROM Reverb 2",  "Par 1", "Par 2", "Par 3"}, {17, 10, 128, 128, 128}, (uint8_t *)rom_rev2},
  {{"Plate Reverb",  "PreD",  "Time",  "Damp" }, {17, 10, 128, 128, 128}, (uint8_t *)K3_V1_2_PLATE},
  {{"St EchoRevrb",  "Delay", "Rep",   "Revrb"}, {17, 10, 128, 128, 128}, (uint8_t *)K3_V1_5_STECHO_REV},

  // Delays
  
  {{"Echo/Reverb",   "Revrb", "Time",  "Level"}, {17, 10, 128, 128, 128}, (uint8_t *)GA_DEMO_ECHO_RPT},
  {{"Pitch Echo",    "Pitch", "Time",  "Mix"  }, {17, 10, 128, 128, 128}, (uint8_t *)rom_pt_echo},
  {{"Rep Echo/Rev",  "Delay", "Rep",   "Revrb"}, {17, 10, 128, 128, 128}, (uint8_t *)K3_V1_4_ECHO_REV},
  {{"Echo/Revrb2",   "Revrb", "Time",  "Level"}, {17, 10, 128, 128, 128}, (uint8_t *)GA_DEMO_ECHO},


  // Modulation
  
  {{"Chorus+Revrb", "Width", "Sweep", "Revrb"}, {17, 10, 128, 128, 128}, (uint8_t *)K3_V1_6_CHOR_REV},
  {{"Flanger+Rev",  "Revrb", "Rate",  "Feed" }, {17, 10, 128, 128, 128}, (uint8_t *)GA_DEMO_FLANGE},

  {{"Phaser+Rev",   "Revrb", "Rate",  "Width"}, {17, 10, 128, 128, 128}, (uint8_t *)GA_DEMO_PHASE},

  {{"Tremolo+Rev",  "Revrb", "Rate",  "Trem" }, {17,  7, 128, 128, 128}, (uint8_t *)rom_trem_rev},

  {{"Vibrato+Rev",  "Revrb", "Rate",  "Width"}, {17, 10, 128, 128, 128}, (uint8_t *)GA_DEMO_VIBRATO},

  {{"Env Filt+Rev", "Revrb", "Sens",  "FiltQ"}, {17, 10, 128, 128, 128}, (uint8_t *)GA_DEMO_WAH},

  {{"Flanger 2",    "Delay", "Rate",  "Width"}, {17, 10, 128, 128, 128}, (uint8_t *)K3_V1_7_FLANGE},
  {{"Flanger+Rev",  "Revrb", "Rate",  "Flang"}, {17, 10, 128, 128, 128}, (uint8_t *)rom_fla_rev},
  {{"Pitch Shift",  "Par 1", "Par 2", "Par 3"}, {17, 10, 128, 128, 128}, (uint8_t *)rom_pitch},

};

// The space for printing algo number on screen is two characters
// In practice, about 80 algorithms is the max that fits on atmega328 32k memory
const uint8_t NALGO = min(99, sizeof(algodata) / sizeof(AlgoDatum));

// Algorithm will be copied to RAM before sending because PROGMEM is too slow
uint8_t algo_buffer[512];

// The strings of the currently visible algorithm are copied to ram
Description current_algo_strings;

/************** Patch data structures *****************/

typedef struct {
  uint8_t algorithm; // index
  uint8_t vol;
  uint8_t mix;
  uint8_t par1;
  uint8_t par2;
  uint8_t par3;
} Patch;

// 5 RGB 565 colours for each patch, from dark to ligth
const uint16_t patch_colors [][5] = {
{0x5940, 0xD1A0, 0xFC20, 0xFD80, 0xFE85},   // Amber
{0x0220, 0x0C80, 0x3640, 0x4F29, 0xA7D4},   // Green
{0x7820, 0xE060, 0xFA8A, 0xFBCF, 0xFDD7},   // Red
{0x31E8, 0x4ACC, 0x638F, 0xA577, 0xC639},   // Gray
{0x00C9, 0x0230, 0x0457, 0x965B, 0xDF5E},   // Blue
{0x3807, 0x8811, 0xF81F, 0xFC5F, 0xFEDF},   // Mag
{0x8285, 0x9307, 0xBBE9, 0xD52D, 0xFFFF},   // Brown
{0x1112, 0x73FD, 0xBD5F, 0x36BC, 0xAFDF},   // Cyan
};

const uint8_t NPATCH = sizeof(patch_colors) / sizeof(patch_colors[0]);

Patch patch_data[NPATCH]; // Keep all patch data in memory, store and restore in EEPROM

const uint16_t P_IN_USE_EE_ADDR = 0;
const uint16_t PATCH_EE_ADDR = 1;
const uint16_t ALGO_EE_ADDR = 512;

uint8_t changed[NPATCH]; // to track what was changed

// State machine for user interface
enum EditState {None, Select, Change};

EditState es = None;
uint8_t selection = 5;                  // default start from line 5
uint8_t patches_in_use = 2;
uint8_t current_patch = 0;
uint16_t bg_col = BLACK;                // gb_col will be light green if patch is edited but not saved


/************** Digipot code ****************/

// vol goes from 0 to 20
// 17 is unity gain (corresponding to pot volume 60)

// mix goes from 0 to 20
// 10 is 1:1 mix max wet - max dry 
// below 10 is less wet - max dry
// above 10 is max wet  - less dry
uint8_t dry_from(uint8_t vol, uint8_t mix)
{  
  if(mix > 20)
    mix = 20;

  if(vol > 20)
    vol = 20;

  if(vol == 0 || mix == 20)
    return 0;  // dry is muted

  uint8_t nominal_vol = 43 + vol; // Max 63

  if(mix <= 10)
    return nominal_vol; // dry is maxed

  uint8_t vol_adjustment = mix - 10;
  if(vol_adjustment > nominal_vol)
    return 0;           // dry is muted

  return nominal_vol - vol_adjustment;
}

uint8_t wet_from(uint8_t vol, uint8_t mix)
{

  if(mix > 20)
    mix = 20;

  if(vol > 20)
    vol = 20;

  if(vol == 0 || mix == 0)
    return 0;  // wet is muted

  uint8_t nominal_vol = 43 + vol; // Max 63

  if(mix >= 10)
    return nominal_vol; // wet is maxed

  uint8_t vol_adjustment = 10 - mix;
  if(vol_adjustment > nominal_vol)
    return 0;           // wet is muted

  return nominal_vol - vol_adjustment;
}


/* Run only once during factory default setup.
 * 
 */
bool pot_init()
{  
  digitalWrite(POT_ENABLE_PIN, LOW); // enable pot communication

  if(!i2c_start(DS1881_write_address))
    return false; // no reply from device
    
  i2c_write(0b10000010);       // Nonvolatile writes: Zero crossing, 63 values + mute
  i2c_stop();
  //delay(20); // wait for EEPROM write to finish
  delay(10); // half delay when clocking nano to 8 MHz

  i2c_start(DS1881_write_address);
  i2c_write(0b10000110);       // Go back to volatile writes
  i2c_stop();
  //delay(20); // wait for EEPROM write to finish
  delay(10); // half delay when clocking nano to 8 MHz

  digitalWrite(POT_ENABLE_PIN, HIGH);

  return true;
}

/* Set the digipot to given value.
 * Min volume 0: -80db attenuation, 
 * Max volume 63:  0db attenuation
 */
bool pot_set(uint8_t pot, uint8_t val)
{
  val &= 0b00111111; // only the low 6 bits matter

  val = 63 - val;    // invert the value.
                     // pot acts as attenuator: 0 - no attenuation (max volume)
                     // we want larger number to mean louder

  if(pot)
    val |= 0b01000000; // set the pot bit;

  digitalWrite(POT_ENABLE_PIN, LOW); // enable pot communication

  if(!i2c_start(DS1881_write_address))
    return false;  // no reply from device
    
  bool res = i2c_write(val);
  i2c_stop();

  digitalWrite(POT_ENABLE_PIN, HIGH);

  return res;
}


/*************** User Interface code **********************/

Button select_button = Button();
Button save_button = Button();
Button fast_button = Button();
Button footswitch = Button();

void draw_selection(const uint16_t* colors, uint8_t pos, bool emph)
{
  uint8_t y = pos * 9;
  uint16_t col = emph ? colors[4] : colors[2];

  draw_rect(0 , y, 96 , 10, col, NONE, false);
}


void clear_selection(uint8_t pos)
{
  uint8_t y = pos * 9;

  draw_rect(0 , y, 96 , 10, bg_col, NONE, false);
}


void draw_patch ()
{
  draw_rect(0, 54, 96, 10, bg_col, bg_col); // clear algorithm name area with background color

  //uint8_t namelen = strlen(current_algo_strings.algoname);
  //uint8_t titlepos = 48 - 3 * namelen; // justify middle

  char buff[3];              // 3 chars + \0
  buff[0] =  ' ';            // put spaces in tens and hundreds position

  byte algo = patch_data[current_patch].algorithm + 1;
  btoa(algo, buff + 3); // give pointer to just after buffer (end)

  // Print algo number
  plot_text(83, 6 * 9 + 1, buff, patch_colors[current_patch][4]);

  // Print algo name
  plot_text(2, 6 * 9 + 1, current_algo_strings.algoname, patch_colors[current_patch][4]);

  const uint16_t * palette = patch_colors[current_patch];
  
  clear_selection(5);
  draw_volume(palette, bg_col, 5, patch_data[current_patch].vol);  
  clear_selection(4);
  draw_mix(palette, bg_col, 4, patch_data[current_patch].mix);   
  clear_selection(3);
  draw_parameter(palette, bg_col, current_algo_strings.par1name, 3, patch_data[current_patch].par1);
  clear_selection(2);
  draw_parameter(palette, bg_col, current_algo_strings.par2name, 2, patch_data[current_patch].par2); 
  clear_selection(1);
  draw_parameter(palette, bg_col, current_algo_strings.par3name, 1, patch_data[current_patch].par3);
}

void draw_panel()
{
  uint8_t boxlen = 96 / NPATCH;

  draw_rect(1, 1, 94, 8, bg_col, bg_col);      
  
  for(uint8_t i = 0; i < patches_in_use; ++i) {
    if(i == current_patch)
      draw_rect(boxlen * i , 1, boxlen , 8, patch_colors[i][2], patch_colors[i][3]);      
    else
      draw_rect(boxlen * i , 1, boxlen , 8, bg_col, patch_colors[i][0]);      
  }
}


void change_selection(const uint16_t * palette, int8_t dir)
{
  if(dir < -1) dir = -1;
  if(dir > 1) dir = 1;
  
  uint8_t oldsel = selection;
  selection += dir + 7;
  selection %= 7;
  clear_selection(oldsel); 
  draw_selection(palette, selection, false);
}

void set_patch()
{
  // Turn digipots
  uint8_t dry = dry_from(patch_data[current_patch].vol, patch_data[current_patch].mix);
  uint8_t wet = wet_from(patch_data[current_patch].vol, patch_data[current_patch].mix);
  
  pot_set(0, dry);  
  pot_set(1, wet);

  // Set analog signal to FV-1
  analogWrite(PAR1_PWM_PIN, patch_data[current_patch].par1);
  analogWrite(PAR2_PWM_PIN, patch_data[current_patch].par2);
  analogWrite(PAR3_PWM_PIN, patch_data[current_patch].par3);
}

void change_patch(int8_t dir)
{
  current_patch += dir + patches_in_use;
  current_patch %= patches_in_use;
  
  bg_col = changed[current_patch] ? MARKED : BLACK; 

  // Save current patch to EEPROM?
  // 100000 writes promised -> 1000 days 100 patch changes per day
  // Maybe not worth it...

  send_algo();
  set_patch();

  // copy current patch strings for ui
  uint8_t algo = patch_data[current_patch].algorithm;
  memcpy_P(&current_algo_strings, &(algodata[algo].desc), sizeof current_algo_strings);
  
  // print changed gui
  draw_panel();
  draw_patch();  
}

// Valid values for dir are -1 and 1
void edit_algo(int8_t dir, bool fast)
{ 
  if(fast)
    dir *= 10;
  
  patch_data[current_patch].algorithm += dir + NALGO;
  patch_data[current_patch].algorithm %= NALGO;

  uint8_t algo = patch_data[current_patch].algorithm;

  // Copy current patch strings for ui
  memcpy_P(&current_algo_strings, &(algodata[algo].desc), sizeof current_algo_strings);

  // Set params to default values. Copy 5 bytes from EEPROM
  Default tmp;
  EEPROM.get(ALGO_EE_ADDR + algo * sizeof(Default),  tmp);

  patch_data[current_patch].vol  = tmp.vol;
  patch_data[current_patch].mix  = tmp.mix;
  patch_data[current_patch].par1 = tmp.par1;
  patch_data[current_patch].par2 = tmp.par2;
  patch_data[current_patch].par3 = tmp.par3;

  changed[current_patch] = true;

  send_algo();
  set_patch();
  
  // print updated elements
  draw_patch();

  const uint16_t * palette = patch_colors[current_patch];
  draw_selection(palette, selection, true);
}


void edit_vol(int8_t dir, const uint8_t pos)
{
  uint8_t vol = patch_data[current_patch].vol;

  if(vol == 0 && dir < 0)
    return; // At minimum
  if(vol == 20 && dir > 0)
    return; // Maxed out

  vol += dir;

  // Change the patch
  patch_data[current_patch].vol = vol;
  changed[current_patch] = true;

  // Turn the digipots.
  uint8_t dry = dry_from(vol, patch_data[current_patch].mix);
  uint8_t wet = wet_from(vol, patch_data[current_patch].mix);
  pot_set(0, dry);
  pot_set(1, wet);

  // Draw the change
  const uint16_t * palette = patch_colors[current_patch];
  draw_volume(palette, bg_col, pos, vol);
}

void edit_mix(int8_t dir, const uint8_t pos)
{
  uint8_t mix = patch_data[current_patch].mix;
   
  if(mix ==  0 && dir < 0)
    return;  // At minimum
  if(mix == 20 && dir > 0)
    return;  // At maximum

  mix += dir;

  // Change the patch
  patch_data[current_patch].mix = mix;
  changed[current_patch] = true;

  // Turn the digipots.
  uint8_t dry = dry_from(patch_data[current_patch].vol, mix);
  uint8_t wet = wet_from(patch_data[current_patch].vol, mix);
  pot_set(0, dry);
  pot_set(1, wet);

  // Draw the change
  const uint16_t * palette = patch_colors[current_patch];
  draw_mix(palette, bg_col, pos, mix);  
}

// Valid values for dir are -1 and 1
void edit_par(int8_t dir, bool fast, uint8_t pin, uint8_t &val, const char *parname, const uint8_t pos)
{
  if(fast)
    dir *= 10;
    
  if(dir < 0 && val < -dir)
    val = 0;
  else if (dir > 0 && (255 - val) < dir)
    val = 255;
  else
    val += dir;

  const uint16_t * palette = patch_colors[current_patch];

  changed[current_patch] = true;

  analogWrite(pin, val);
  draw_parameter(palette, bg_col, parname, pos, val);
}

void edit_menubar(int8_t dir)
{
  if(dir < -1) dir = -1;
  if(dir > 1) dir = 1;

  // patches_in_use must be in [2, NPATCH]
  if(patches_in_use <= 2 && dir < 0)
    return;
  if(patches_in_use >= NPATCH && dir > 0)
    return;

  const uint16_t * palette = patch_colors[current_patch];

  patches_in_use += dir;

  if(current_patch >= patches_in_use) {
    change_patch(-1);  // move to previous patch since current is not in use
    draw_selection(palette, selection, true);
  } else {
    draw_panel();
    draw_selection(palette, selection, true);
  }
  
  // write number of patches in use to eeprom
  EEPROM.put(P_IN_USE_EE_ADDR, patches_in_use);
}

void edit_patch(uint8_t sel, int8_t dir, bool fast)
{
  switch(sel) {
  case 6 : edit_algo(dir, fast);     break;
  case 5 : edit_vol(dir, sel); break;
  case 4 : edit_mix(dir, sel); break;
  case 3 : edit_par(dir, fast, PAR1_PWM_PIN, patch_data[current_patch].par1, current_algo_strings.par1name, sel); break;
  case 2 : edit_par(dir, fast, PAR2_PWM_PIN, patch_data[current_patch].par2, current_algo_strings.par2name, sel); break;
  case 1 : edit_par(dir, fast, PAR3_PWM_PIN, patch_data[current_patch].par3, current_algo_strings.par3name, sel); break;
  case 0 : edit_menubar(dir); break;
  }
}

void save_current_patch()
{
  uint16_t eeAddress = PATCH_EE_ADDR + current_patch * sizeof(Patch);
  Patch curr = patch_data[current_patch];
  
  EEPROM.put(eeAddress, curr);

  // Save to algo defaults also so that next time they can be used
  
  eeAddress = ALGO_EE_ADDR + curr.algorithm * sizeof(Default);
  Default tmp = {curr.vol, curr.mix, curr.par1, curr.par2, curr.par3};
  
  EEPROM.put(eeAddress, tmp);
      
  changed[current_patch] = false;
  bg_col = BLACK;
  draw_panel();
  draw_patch();             // Redraw the whole screen
}



/********** 24LC32A EEPROM emulation */

/* Compression algorithm based on
 * http://excamera.com/sphinx/article-compression.html
 *  
 * With small modifications.
 */
class Flashbits {
public:

  void begin(prog_uchar *s) 
  {
    src = s;
    mask = curr_byte = 0x00;
  }

  uint8_t get1(void)
  {
    if (!mask) {
      mask = 0x80;
      curr_byte = pgm_read_byte_near(src++);
    }

    uint8_t r = (curr_byte & mask) != 0;
    mask >>= 1;

    return r;
  }

  uint16_t getn(uint8_t n) 
  {  
    uint16_t r = 0;
    while (n--) {
      r <<= 1;
      r |= get1();
    }
    return r;
  }
  
private:
  prog_uchar *src;
  uint8_t mask;
  uint8_t curr_byte;
};

Flashbits bits;

void decompress(uint8_t *dst, prog_uchar *src)
{
  bits.begin(src);
  uint8_t O = bits.getn(4);
  uint8_t L = bits.getn(4);
  uint8_t M = bits.getn(2);
  
  uint8_t *end = dst + bits.getn(16);

  while (dst != end) {
    if (bits.get1() == 0) {
      *dst++ = bits.getn(8);
    } else {
      int offset = -bits.getn(O) - 1;
      int length = bits.getn(L) + M;
      while (length--) {
        *dst = *(dst + offset);
        dst++;
      }
    }
  }
}

void send_algo()
{
  // Copy FV-1 algorithm from PROGMEM to ram buffer.
  // Progmem is too slow in the send loop.
  // Since we will anyway have to copy from PROGMEM,
  // it makes sense to compress the algorithms and get
  // space savings (almost 2x).
  
  uint8_t algo = patch_data[current_patch].algorithm;
  decompress(algo_buffer, (prog_uchar *)pgm_read_word(&(algodata[algo].prog_addr)));

  // Set up everything to be ready
  const uint8_t sda_mask = (1 << PIND5);
  const uint8_t clk_mask = (1 << PIND6);
  uint8_t prev_clk = clk_mask;

  DDRD &= ~(clk_mask);   // clk pin as input
  DDRD &= ~(sda_mask);   // sda pin as input
  PORTD &= ~(sda_mask);  // SDA pin is pulled low or floated up 
                         // -> we need to set pin low only once.
                         //    Toggling is done by setting pin to
                         //    input (let float high) or output (pull low).
                         //    FV-1 internal pullups are ok (tested).

  uint16_t pos = 0;
  uint8_t curr_byte = algo_buffer[pos];  
  uint8_t bit_mask = 0b10000000;
  uint8_t clk_count = 0;

  // Overclock (3.3V 16 MHz)
  clock_prescale_set(clock_div_1);

  // Undivided attention for FV-1 requests
  noInterrupts();

  // Notify FV-1 of patch change by toggling the notify pin
  PIND = _BV(4);   // Toggle pin 4
  
  while(clk_count < 37)              // Handle the header
  {
    uint8_t clk = PIND & clk_mask;

    if(!clk && prev_clk) {           // scl went down
      
      switch(clk_count)
      {
        case 8:
        case 17:
        case 26:
        case 36:
          DDRD  |= sda_mask;         // send ACK - pull sda pin low
          break;
        default:
          DDRD &= ~(sda_mask);       // Release
          break;    
      }      
      clk_count++;
    }
    prev_clk = clk;
  }
 
  clk_count = 0;
  while(pos < 512)                   // Send the data
  {
    uint8_t clk = PIND & clk_mask;
    
    if(!clk && prev_clk) {           // scl went down

      if(clk_count != 8) {           // Sending byte
        
        if (curr_byte & bit_mask) { 
          DDRD &= ~(sda_mask);       // Send 1 = Let High
        } else {
          DDRD  |= sda_mask;         // Send 0 = Pull Low
        }        
        bit_mask >>= 1;
        clk_count++;
        
      } else {                       // Let reciever ACK
        
        DDRD &= ~(sda_mask);         // Release
        clk_count = 0;
        bit_mask = 0b10000000;
        pos++;
        curr_byte = algo_buffer[pos];      
      }
    }
    prev_clk = clk;
  }

  interrupts();

  // Restore 8 MHz
  clock_prescale_set(clock_div_2);
}


void setup()
{
  // Reduce speed by half to 8 MHz by setting clock prescaler to 2,
  // since we are running with 16 MHz crystal at 3.3 V
  clock_prescale_set(clock_div_2);

  // Use 9600 baud in monitor... half speed
  //Serial.begin(19200);

  // set up pins

  // Use high frequency PWM
  TCCR1A = (TCCR1A & 0b11111100) | 0b00000001;    // set timer 1 to 8 bit mode
  TCCR1B = (TCCR1B & 0b11100000) | 0b00001001;    // set timer 1 to fast PWM, divisor to 1 for PWM frequency of 31372.55 Hz

  TCCR2A = (TCCR2A & 0b11111100) | 0b00000011;    // set timer 2 to fast PWM
  TCCR2B = (TCCR2B & 0b11111000) | 0b00000001;    // set timer 2 divisor to 1 for PWM frequency of 31372.55 Hz 

  // PWM outputs
  pinMode(PAR1_PWM_PIN, OUTPUT);
  pinMode(PAR2_PWM_PIN, OUTPUT);
  pinMode(PAR3_PWM_PIN, OUTPUT);

  // 24LC32 EEPROM emulation
  pinMode(PC_NOTIFY_PIN, OUTPUT);
  pinMode(I2C_EEPROM_EMUL_SDA_PIN, INPUT);
  pinMode(I2C_EEPROM_EMUL_SCL_PIN, INPUT);

  // Buttons and encoder
  select_button.attach(SELE_BUT_PIN, INPUT);
  select_button.interval(5);
  save_button.attach(SAVE_BUT_PIN, INPUT);
  save_button.interval(5);
  fast_button.attach(FAST_BUT_PIN, INPUT);
  fast_button.interval(5);
  footswitch.attach(FOOT_BUT_PIN, INPUT);
  footswitch.interval(5);

  pinMode(ENC1_PIN, INPUT_PULLUP); 
  pinMode(ENC2_PIN, INPUT_PULLUP); 
  
  // SoftI2C is used for digipot control
  pinMode(POT_ENABLE_PIN, OUTPUT); 
  i2c_init(); // Uses hardware I2C (pins A4 and A5)

  
  // OLED Display setup
  init_display(); // Uses pins 11, 12 and 13
  clear_display();
  display_on(true);


  // Write factory defaults to EEPROM if requested
  // SAVE + SELECT pressed at boot for 2 sec
  unsigned long startTime = millis();
  unsigned long heldTime = 0;
  while (digitalRead(SELE_BUT_PIN) == HIGH && digitalRead(SAVE_BUT_PIN) == HIGH)// && digitalRead(FAST_BUT_PIN) == HIGH)
  {
   heldTime = millis() - startTime;
   if (heldTime > 1000)
     break;
  }
      
  if(heldTime > 1000) {

    // Digipot initial settings, saved in its EEPROM.
    // Depends on i2c_init(). Hangs setup if no pot present...
    pot_init();

    // Write algorithm default settings
    Default tmp_d;
    for(uint8_t i = 0; i < NALGO; ++i) {
      memcpy_P(&tmp_d, &(algodata[i].def), sizeof(Default));
      EEPROM.put(ALGO_EE_ADDR + i * sizeof(Default), tmp_d);
      delay(10); 
    }
  
    // Write empty patches using the first algorithm and default settings
    memcpy_P(&tmp_d, &(algodata[0].def), sizeof(Default));   
    Patch tmp_p = {0, tmp_d.vol, tmp_d.mix, tmp_d.par1, tmp_d.par2, tmp_d.par3};
    
    for(uint8_t i = 0; i < NPATCH; ++i) {
      EEPROM.put(PATCH_EE_ADDR + i * sizeof(Patch), tmp_p);
      delay(10); 
    }
    
    // Set 8 patches as default
    EEPROM.put(P_IN_USE_EE_ADDR, (uint8_t)8);  
  }
    
  // load patch settings from EEPROM
  uint16_t eeAdress = PATCH_EE_ADDR; 
  for(uint8_t i = 0; i < NPATCH; ++i) {
    EEPROM.get(eeAdress, patch_data[i]);
    if(patch_data[i].algorithm >= NALGO) {
      patch_data[i].algorithm = 0;   // something wrong with algo index, set to zero 
      changed[i] = true;             // user will have to edit and save
    }
    if(patch_data[i].vol > 20)
      patch_data[i].vol = 20;
    if(patch_data[i].mix > 100)
      patch_data[i].mix = 100;
    eeAdress += sizeof(Patch);
  }
  
  current_patch = 0;
  
  // get patches_in_use from EEPROM
  EEPROM.get(P_IN_USE_EE_ADDR, patches_in_use);
  if(patches_in_use < 2)
    patches_in_use = 2;
  if(patches_in_use >= NPATCH)
    patches_in_use = NPATCH;

  // copy current patch strings for ui
  uint8_t algo = patch_data[current_patch].algorithm;
  memcpy_P(&current_algo_strings, &(algodata[algo].desc), sizeof current_algo_strings);

  // send current patch algorithm to FV-1
  send_algo();
  set_patch();

  // print initial gui
  draw_panel();
  draw_patch();

  //Serial.println("Setup finished");
  //Serial.flush();

}


/* Rotary encoder handling code from:
 * https://www.best-microcontroller-projects.com/rotary-encoder.html
 */
uint8_t prevNextCode = 0;
uint16_t store=0;

// A valid CW or CCW move returns 1 or -1, invalid returns 0.
int8_t read_rotary() {
  const int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode <<= 2;
  if (digitalRead(ENC1_PIN)) prevNextCode |= 0x02;
  if (digitalRead(ENC2_PIN)) prevNextCode |= 0x01;
  prevNextCode &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[prevNextCode] ) {
      store <<= 4;
      store |= prevNextCode;
      //if (store==0xd42b) return 1;
      //if (store==0xe817) return -1;
      if ((store&0xff)==0x2b) return -1;
      if ((store&0xff)==0x17) return 1;
   }
   return 0;
}


void loop()
{
  const uint16_t * palette = patch_colors[current_patch];
  int8_t dir = read_rotary();

  select_button.update();
  save_button.update();
  fast_button.update();
  footswitch.update();

  // Select button pressed
  if(select_button.pressed()) {
    if(es == None) {
      es = Select;
      draw_selection(palette, selection, false);
    } else if (es == Select) {
      es = Change;
      // emphasize bar that is being edited
      draw_selection(palette, selection, true);
    } else if (es == Change) {
      es = None;
      if(changed[current_patch]) {
        bg_col = MARKED;
        draw_panel();
        draw_patch();             // Redraw the whole screen
      }
      clear_selection(selection);
    }
  }

  // Encoder turned
  if(dir) {    
    if(es == None) { // first movement activates Select mode
      es = Select;
      draw_selection(palette, selection, false); 
    } else if(es == Select) {               
      change_selection(palette, dir);  // Move selection
    } else if (es == Change) {
      bool fast = (fast_button.read() == HIGH);
      edit_patch(selection, -dir, fast);       // Change parameter
    }
  }

  // Save button pressed
  if(save_button.pressed()) {
    if(changed[current_patch]) {
      save_current_patch();
      es = None;                               // Go to default mode
      clear_selection(selection);
    }
  }

  // Footswitch pressed
  if(footswitch.pressed()) {
    if(es == Select) {
      clear_selection(selection);
      es = None;
    }
    if (es != Change) {  // Patch change disabled during editing
      change_patch(1);
    }
  }
} 
