
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
// classes and functions for switches
//

#include <Streaming.h>
#include "defs.h"
#include "SwitchProc.h"

//
/// a class to encapsulate a physical switch -- with non-blocking processing
//

nbSwitch::nbSwitch(byte pin, byte pressedState = LOW) {

  // construct and initialise the switch object
  // default to active low, ie normally high and pulled low when pressed

  this->_pin = pin;
  this->_pressedState = pressedState;

  this->reset();
}

void nbSwitch::run(void) {

  // check for state change

  pinMode(_pin, INPUT_PULLUP);

  // read the pin
  this->_currentState = digitalRead(_pin);

  // has state changed ?
  if (this->_currentState != this->_lastState) {

    // yes -- state has changed since last call to this method
    // Serial << endl << F("  -- switch state has changed state from ") << this->_lastState << " to " << this->_currentState << endl;
    // Serial << F("  -- last state change at ") << this->_lastStateChangeTime << ", diff = " << millis() - this->_lastStateChangeTime << endl;

    this->_lastState = this->_currentState;
    this->_prevStateDuration = this->_lastStateDuration;
    this->_lastStateDuration = millis() - this->_lastStateChangeTime;
    this->_lastStateChangeTime = millis();
    this->_stateChanged = true;
    this->_doubleClick = false;

    if (this->_currentState == this->_pressedState) {
      // it has been pressed -- start the timer
      // Serial << F("  -- switch has been pressed") << endl;
    } else {
      // Serial << F("  -- switch has been released") << endl;

      // double-click detection
      // two clicks of less than 250ms, less than 500ms apart

      // Serial << F("  -- last state duration = ") << this->_lastStateDuration << endl;
      // Serial << F("  -- this release = ") << this->_lastStateChangeTime << F(", last release = ") << this->_prevReleaseTime << endl;

      if ((this->_lastStateChangeTime - this->_prevReleaseTime) < 500 && \
          (this->_lastStateDuration < 250) && (this->_prevStateDuration < 250)) {
        // Serial << F("  -- double click !") << endl;
        this->_doubleClick = true;
      }

      // save release time
      this->_prevReleaseTime = this->_lastStateChangeTime;
    }

  } else {

    // no -- state has not changed
    this->_stateChanged = false;
  }

  return;
}

void nbSwitch::reset(void) {

  // reset state values to defaults -- at start up or anytime later

  if (this->_pressedState == LOW) {
    pinMode(this->_pin, INPUT_PULLUP);
  } else {
    pinMode(this->_pin, INPUT);
  }

  this->_lastState = !this->_pressedState;
  this->_stateChanged = false;
  this->_doubleClick = false;
  this->_lastStateChangeTime = 0UL;
  this->_lastStateDuration = 0UL;
  this->_prevReleaseTime = 0UL;
  this->_prevStateDuration = 0UL;

  return;
}

bool nbSwitch::stateChanged(void) {

  // has switch state changed ?
  // Serial << F("  -- switch state changed = ") << this->_stateChanged << endl;
  return this->_stateChanged;
}

bool nbSwitch::getState(void) {

  // return the current switch state read
  // Serial << F("  -- switch state = ") << this->_currentState << endl;
  return this->_currentState;
}

bool nbSwitch::isPressed(void) {

  // is the switch pressed ?
  // Serial << F("  -- switch pressed ? = ") << (this->_currentState == this->_pressedState) << endl;
  return (this->_currentState == this->_pressedState);
}

bool nbSwitch::isDoubleClick(void) {

  // was the last transition the release of the second click of a double-click sequence ?
  return (this->_doubleClick);
}

unsigned long nbSwitch::getCurrentStateDuration(void) {

  // how long has the switch been in its current state ?
  // Serial << F("  -- current state duration = ") << (millis() - this->_lastStateChangeTime) << endl;
  return (millis() - this->_lastStateChangeTime);
}

unsigned long nbSwitch::getLastStateDuration(void) {

  // how long was the last state active for ?
  // Serial << F("  -- last state duration = ") << this->_lastStateDuration << endl;
  return this->_lastStateDuration;
}

unsigned long nbSwitch::getLastStateChangeTime(void) {

  // when was the last state change ?
  // Serial << F("  -- last state change at ") << this->_lastStateChangeTime << endl;
  return this->_lastStateChangeTime;
}

void nbSwitch::resetCurrentDuration(void) {

  // reset the state duration counter
  this->_lastStateChangeTime = millis();
  return;
}

//
/// a class to encapsulate a rotary encoder
//

/*
   class nbEncoder {

  public:
    nbEncoder(byte pinA, byte pinB, void (*isr)(void), volatile int *counter);
    void begin(void);
    void run(void);
    bool changed(void);
    void reset(void);
    byte getCount(void);
    void setCount(int val);

  private:
    byte _pinA, _pinB;
    bool _changed;
    void (*_isr)(void);
    volatile int *_counter, _lastcount;
  };
*/

extern volatile int encoderCount;

nbEncoder::nbEncoder(byte pinA, byte pinB, void (*isr)(void), volatile int *counter) {

  // constructor
  this->_pinA = pinA;
  this->_pinB = pinB;
  this->_isr = isr;
  this->_counter = counter;

  return;
}

void nbEncoder::begin(void) {

  // one time initialisation
  pinMode(this->_pinA, INPUT_PULLUP);
  pinMode(this->_pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(this->_pinA), this->_isr, FALLING);

  return;
}

void nbEncoder::run(void) {

  // if (this->_lastcount != *(this->_counter)) {
  //   this->_lastcount = *(this->_counter);
  if (this->_lastcount != encoderCount) {
    this->_lastcount = *(this->_counter);
    this->_changed = true;
  } else {
    this->_changed = false;
  }

  return;
}

void nbEncoder::reset(void) {

  // *(this->_counter = 0);
  encoderCount = 0;
  this->_lastcount = 0;
  this->_changed = false;

  return;
}

int nbEncoder::getCount(void) {

  return encoderCount;
}

void nbEncoder::setCount(int val) {

  // *(this->_counter) = val;
  encoderCount = val;
  return;
}

bool nbEncoder::changed(void) {

  return (this->_changed);
}

