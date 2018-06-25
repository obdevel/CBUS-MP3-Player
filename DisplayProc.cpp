
/*

  Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)

  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

//
// control functions for switches, rotary encoder and LED displays
//

#include <LEDControl.h>
#include "defs.h"
#include "DisplayProc.h"
#include "ModuleConfig.h"

//
///  7-segment display class
//

extern struct _nodevars nodevars;

LedControl _lc(LED_DATA_IN, LED_CLK, LED_LOAD, 1);

s7SegDisplay::s7SegDisplay() {

  pinMode(LED_LOAD, OUTPUT);
  pinMode(LED_CLK, OUTPUT);
  pinMode(LED_DATA_IN, OUTPUT);
}

// set intensity / brightness level

void s7SegDisplay::reset(void) {

  _lc.shutdown(0, false);
  _lc.setIntensity(0, nodevars.bright);
  _lc.clearDisplay(0);

  return;
}

void s7SegDisplay::setIntensity(byte level) {

  _lc.setIntensity(0, level);
  return;
}

// display a 2-digit number on the LED display

void s7SegDisplay::displayNumber(byte val) {

  byte tens, units;

  // display numbers greater than 99 as hex
  if (val > 99) {
    displayHex(val);

  } else {

    if (val == 0) {
      tens = 0;
      units = 0;
    } else {
      tens = (val / 10);
      units = val % 10;
    }

    _lc.setDigit(0, 0, tens, false);
    _lc.setDigit(0, 1, units, false);
  }

  return;
}

// display a 2-digit hexadecimal value on the LED display
// illuminate DPs to indicate it's not a decimal number

void s7SegDisplay::displayHex(byte val) {

  // char buf[3];
  byte d0, d1;

  d0 = val / 16;
  d1 = val % 16;

  _lc.setDigit(0, 0, d0, true);
  _lc.setDigit(0, 1, d1, true);

  return;
}

/// display a 2 character text message on the LED display, e.g. 'nc' for 'no card'

void s7SegDisplay::displayText(byte c1, byte c2) {

  _lc.setChar(0, 0, c1, false);
  _lc.setChar(0, 1, c2, false);

  return;
}

//
/// briefly flash a message, alternating letter and numbers, e.g. 'n' for node number or 'e' for error
//

void s7SegDisplay::displayMessage(byte code, byte num) {

  for (byte i = 0; i < 5; i ++) {
    this->displayText(code, code);
    delay(250);
    this->displayNumber(num);
    delay(250);
  }

  delay(500);
  return;
}

// --------------------

//
/// class for individual LED with non-blocking control
//

nbLED::nbLED(uint8_t ledPin) {
  this->_pin = ledPin;
  this->_state = LOW;
  this->_lastState = !this->_state;
  this->_blink = false;
  this->_lastTime = 0L;
  pinMode(this->_pin, OUTPUT);
}

// return the current state, on or off

bool nbLED::getState() {

  return _state;
}

// turn LED state on

void nbLED::on(void) {

  this->_state = true;
  this->_blink = false;
}

// turn LED state off

void nbLED::off(void) {

  this->_state = false;
  this->_blink = false;
}

// toggle LED state from on to off or vv

void nbLED::toggle(void) {

  this->_state = !this->_state;
}

// blink LED

void nbLED::blink() {

  this->_blink = true;
}

// actually operate the LED dependent upon its current state
// must be called frequently from loop()

void nbLED::run() {

  if (this->_blink) {

    // blinking
    if ((millis() - this->_lastTime) > BLINK_RATE) {
      this->toggle();
      this->_lastTime = millis();
    }
  }

  digitalWrite(this->_pin, (this->_state ? HIGH : LOW));

}

