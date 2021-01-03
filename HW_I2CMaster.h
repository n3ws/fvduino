/* Based on :
 *  
 * Arduino SoftI2C library. Uses only the HW implementation.
 *
 * Version 1.4
 *
 * Copyright (C) 2013, Bernhard Nebel and Peter Fleury
 *
 * This is a very fast and very light-weight software I2C-master library 
 * written in assembler. It is based on Peter Fleury's I2C software
 * library: http://homepage.hispeed.ch/peterfleury/avr-software.html
 * Recently, the hardware implementation has been added to the code,
 * which can be enabled by defining I2C_HARDWARE.
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino I2cMaster Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/* TWI hardware is used. You have to use
 * the standard SDA/SCL pins (and, of course, the chip needs to support
 * this).
 *
 * You can also define the following constants (see also below):
 * - I2C_CPUFREQ, when changing CPU clock frequency dynamically
 * - I2C_MAXWAIT = 0..32767 number of retries in i2c_start_wait. 0 means never stop.
 */

#ifndef HW_I2CM_H
#define HW_I2CM_H


#ifndef __AVR_ARCH__
#error "Not an AVR MCU! Use 'SlowSoftI2CMaster'!"
#endif

#include <avr/io.h>
#include <Arduino.h>
#include <util/twi.h>


// You can set I2C_CPUFREQ independently of F_CPU if you 
// change the CPU frequency on the fly. If you do not define it,
// it will use the value of F_CPU
#ifndef I2C_CPUFREQ
#define I2C_CPUFREQ F_CPU
#endif


// I2C_MAXWAIT can be set to any value between 0 and 32767. 
// 0 means no time out.
#define I2C_MAXWAIT 500

#define SCL_CLOCK 100000UL


/* Init function. Needs to be called once in the beginning.
 * Returns false if SDA or SCL are low, which probably means 
 * a I2C bus lockup or that the lines are not pulled up.
 */
bool i2c_init(void)
{
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);

  TWSR = (1<<TWPS0); // prescaler is 4
  TWBR = ((I2C_CPUFREQ/SCL_CLOCK)-16)/8;

  return (digitalRead(SDA) != 0 && digitalRead(SCL) != 0);
}


/* Start transfer function: <addr> is the 8-bit I2C address (including the R/W bit). 
 * Return: true if the slave replies with an "acknowledge", false otherwise
 */
bool  i2c_start(uint8_t addr)
{
  uint8_t   twst;

  // send START condition
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

  // wait until transmission completed
  while(!(TWCR & (1<<TWINT))) {}

  // check value of TWI Status Register. Mask prescaler bits.
  twst = TW_STATUS & 0xF8;
  if ( (twst != TW_START) && (twst != TW_REP_START)) return false;
  
  // send device address
  TWDR = addr;
  TWCR = (1<<TWINT) | (1<<TWEN);
  
  // wail until transmission completed and ACK/NACK has been received
  while(!(TWCR & (1<<TWINT))) { }
  
  // check value of TWI Status Register. Mask prescaler bits.
  twst = TW_STATUS & 0xF8;
  if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return false;
  
  return true;
}


/* Similar to start function, but wait for an ACK! Will timeout if I2C_MAXWAIT > 0.
 */
bool  i2c_start_wait(uint8_t addr)
{
  uint8_t   twst;
  uint16_t  maxwait = I2C_MAXWAIT;
  
  while (true) {
    // send START condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    // wait until transmission completed
    while(!(TWCR & (1<<TWINT))) {}
    
    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    // send device address
    TWDR = addr;
    TWCR = (1<<TWINT) | (1<<TWEN);
    
    // wail until transmission completed
    while(!(TWCR & (1<<TWINT))) {}
    
    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) {    	    
      // device busy, send stop condition to terminate write operation
      TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
      // wait until stop condition is executed and bus released
      while(TWCR & (1<<TWSTO)) {}

      if (maxwait)
        if (--maxwait == 0)
          return false;
	
      continue;
    }
    //if( twst != TW_MT_SLA_ACK) return 1;
    return true;
  }
}


/* Issue a stop condition, freeing the bus.
 */
void i2c_stop(void)
{
  // send stop condition
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
  
  // wait until stop condition is executed and bus released
  while(TWCR & (1<<TWSTO)) {}
}

/* Write one byte to the slave chip that had been addressed
 * by the previous start call. <value> is the byte to be sent.
 * Return: true if the slave replies with an "acknowledge", false otherwise
 */
bool i2c_write(uint8_t value)
{	
  uint8_t   twst;

  // send data to the previously addressed device
  TWDR = value;
  TWCR = (1<<TWINT) | (1<<TWEN);
  
  // wait until transmission completed
  while(!(TWCR & (1<<TWINT))) {}
  
  // check value of TWI Status Register. Mask prescaler bits
  twst = TW_STATUS & 0xF8;
  if( twst != TW_MT_DATA_ACK) return false;
  return true;
}


/* Read one byte. If <last> is true, we send a NAK after having received 
 * the byte in order to terminate the read sequence. 
 */
uint8_t i2c_read(bool last)
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (last ? 0 : (1<<TWEA));
  while(!(TWCR & (1<<TWINT))) {}
  
  return TWDR;
}

#endif
