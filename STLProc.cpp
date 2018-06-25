
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

// Sound to light class

#include "defs.h"
#include "STLProc.h"


STL::STL(byte inpin, byte outpin) {

  // constructor

  _inpin = inpin;
  _outpin = outpin;

  return;
}

void STL::init(void) {

  // do initialisation actions

  pinMode(_inpin, INPUT);
  pinMode(_outpin, OUTPUT);
  digitalWrite(_outpin, LOW);

  _val = 0;
  _threshhi = 0;
  _threshlo = 0;
  _high = 0;
  _low = 0;

  return;
}

void STL::run(void) {

  // are we enabled ?
  if (!_state) {
    return;
  }

  // read the audio output value
  _val = analogRead(_inpin);
  // Serial << F("> sensor = ") << _val << endl;

  // set extremes of input we've seen
  if (_val < _low) _low = _val;
  if (_val > _high) _high = _val;

  // above high threshold ?
  if (_val >= _threshhi) {
    // Serial << F("> sound level = ") << _val << F(" above threshold = ") << _threshi << endl;
    digitalWrite(_outpin, HIGH);
  }

  // below low threshold ?
  if (_val <= _threshlo) {
    // Serial << F("> sound level = ") << _val << F(" below threshold = ") << threshlo << endl;
    digitalWrite(_outpin, LOW);
  }

  return;
}

void STL::enable(bool state) {

  _state = state;

  if (!_state) {
    digitalWrite(_outpin, LOW);
  }
  
  return;
}

void STL::setThresholdHigh(unsigned int level) {

  _threshhi = level;
  return;
}

void STL::setThresholdLow(unsigned int level) {

  _threshlo = level;
  return;
}

unsigned int STL::getThresholdHigh(void) {

  return _threshhi;
}

unsigned int STL::getThresholdLow(void) {

  return _threshlo;
}

unsigned int STL::getHighest(void) {

  return _high;
}

unsigned int STL::getLowest(void) {

  return _low;
}

unsigned int STL::getLast(void) {

  return _val;
}

bool STL::getState(void) {

  return _state;
}


