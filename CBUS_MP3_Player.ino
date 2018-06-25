
//
/// CBUS_MP3_Player_v2.ino
//

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

/*
      3rd party libraries needed for compilation: (not for binary-only distributions)

      Streaming           -- C++ stream style output, v 5, (http://arduiniana.org/libraries/streaming/)
      DFRobotDFPlayerMini -- DFPlayer Mini control, v 1.0.2  (https://github.com/DFRobot/DFRobotDFPlayerMini)
      LedControl          -- MAX7219 / 7-seg LEDs, v 1.0.6 (https://github.com/wayoda/LedControl)
      MCP_CAN             -- CAN Bus library using MCP2515, v 1.5 (https://github.com/coryjfowler/MCP_CAN_lib)
      extEEPROM           -- I2C EEPROM library, v 3.4.0, https://github.com/JChristensen/extEEPROM
*/



// standard Arduino libraries
#include <Wire.h>
#include <EEPROM.h>

// 3rd party libraries
#include <Streaming.h>
#include <DFRobotDFPlayerMini.h>
#include <extEEPROM.h>

// project header files
#include "defs.h"
#include "CBusMessageBuffer.h"
#include "CBusMessageProc.h"
#include "ModuleConfig.h"
#include "SwitchProc.h"
#include "DisplayProc.h"
#include "MP3PlayerProc.h"
#include "STLProc.h"
#include "bootload.h"

// MERG CBUS definitions
#include "cbusdefs.h"

// constants
const byte VER_MAJ = 2;                  // code major version
const char VER_MIN = 'a';                // code minor version
const byte VER_BETA = 0;                 // code beta sub-version

// pin assignments
const byte ENC0 = 3;                     // encoder pin A -- digital pin 3 = interrupt INT1
const byte ENC1 = 9;                     // encoder pin B
const byte DEBOUNCE_PERIOD = 15;         // encoder debounce guard time (in mS)
const byte MINCOUNT = 0;                 // minimum display value
const byte MAXCOUNT = 255;               // maximum display value

const byte CAN0_INT = 2;                 // MCP2515 /INT pin
const byte CAN0_CS = SS;                 // MCP2515 SPI CS = SS (pin 10)

const byte STL_IN  = A2;                 // ADC audio input from player's speaker output
const byte STL_OUT = A3;                 // drives MOSFET gate - for external StL power switching

const byte SW0 = A5;                     // push button switch pin

const byte MAX_SEQ_LEN = 64;
const byte MAX_TRACK_NUM = 254;

// runtime control
const byte PL_CMD_START = 240;
const byte PL_VOL_UP    = 240;
const byte PL_VOL_DN    = 241;
const byte PL_SKIP      = 242;
const byte PL_ESTOP     = 243;
const byte PL_PAUSE     = 244;
const byte PL_MUTE      = 245;
const byte PL_UNMUTE    = 246;
const byte STL_ON       = 247;
const byte STL_OFF      = 248;
const byte PL_LOOP_MODE = 249;
const byte PL_VOL_ABS   = 250;
const byte PL_IMMED     = 251;
const byte PL_HOLD      = 252;
const byte PL_UNHOLD    = 253;
const byte PL_EV_CHAIN  = 254;
const byte EV_NULL      = 255;

//
/// forward function declarations
//

byte CBUSEventProc(void);
void loadNVs(void);
void doReset(void);
void doCANEnum(void);
void initFLiMProc(void);
void revertSLiMProc(void);
void doSLiMLearn(void);
void doPlayOrStop(void);
void doPlayerCommand(byte track, byte val);
void printConfig(void);
void encoderPinA_ISR(void);
void upload(void);
void encoderPinA_ISR(void);

// node parameters -- we define 20 params (1-20) plus 0 = number of params
unsigned char params[21] = {

  20,              //  0 num params = 10
  0xa5,            //  1 manf = MERG, 165
  VER_MIN,         //  2 code minor version
  0x07,            //  3 module id = CANLED64, 7
  EE_MAX_EVENTS,   //  4 num events
  EE_NUM_EVS,     //  5 num evs per event
  EE_NUM_NVS,      //  6 num NVs
  VER_MAJ,         //  7 code major version
  0x05,            //  8 flags = 5, FLiM, consumer
  0x32,            //  9 processor id = 50
  PB_CAN,          // 10 interface protocol = CAN, 1
  0x00,
  0x00,
  0x00,
  0x00,
  '3',
  '2',
  '8',
  'P',
  CPUM_ATMEL,      // 19
  VER_BETA         // 20
};

unsigned char mname[7] = { 'S', 'O', 'U', 'N', 'D', ' ', ' ' };

// node variables
struct _nodevars nodevars;

// CAN/CBUS
MCP_CAN _CAN0(CAN0_CS);
CANBus CBUS;
cbusMessageBuffer _messageBuffer;

// MP3 player
MP3Player mp3Player;
byte track = 0;
byte playing = false, paused = false, muted = false;
unsigned long startPlay = 0L, duration = 0L, lastCompleteTime = 0L;

// rotary encoder
volatile int encoderCount = 0;
nbEncoder enc0(ENC0, ENC1, encoderPinA_ISR, &encoderCount);

// dual 7-seg display
s7SegDisplay s7seg = s7SegDisplay();

// individual LEDs
nbLED ledGrn(LED_GRN);
nbLED ledYlw(LED_YLW);

// pushbutton switch
nbSwitch pbSwitch(SW0, LOW);       // active low -- signal is connected to 0V when pressed

// sound to light
STL stl(STL_IN, STL_OUT);

// external I2C EEPROM
extEEPROM extmem(kbits_512, 1, 64, 0x50);    // 512Kbits max, 64byte pagesize, at I2C address 0x50

// global variables related to playing and learning
bool bFLiM = false, bLearn = false, bLearnMode = 1, bModeChanging = false, bEVChain = false;
bool bCANenum = false, bCANenumComplete = true, bInSequence = false, bLoopMode = false, bOnHold = false;

byte trackSequence[MAX_SEQ_LEN], wseqno = 0, rseqno = 0, seqadd = 0, lastTrackFinished = 0;
byte lastVol = 5, lastOpc = 0, chainEvt = 255, lastevtidx = 0;
char msgstr[100], dstr[50];
signed int busyVal = 0, lastBusyVal = 0;

unsigned long CANenumTime = 0L, timeOutTimer = 0L;
unsigned int nodeNum = 0;
byte CANID = 0, idChoice = 0, highCANID = 0, enums = 0;
byte enumResults[16];               // 128 bits for storing CAN ID enumeration results
CANFrame msg;

//
/// rotary encoder interrupt handler (ISR)
//

void encoderPinA_ISR(void) {

  byte curr0, curr1;
  static unsigned long lasttime = millis() - (DEBOUNCE_PERIOD + 5);   // must be > guard time or first interrupt will be ignored !!

  if (millis() - lasttime < DEBOUNCE_PERIOD) {
    // called within the debounce guard time -- no action
    return;
  }

  lasttime = millis();
  curr0 = digitalRead(ENC0);
  curr1 = digitalRead(ENC1);

  // this should never be true, as the interrupt triggers on a falling edge
  if (curr0 != 0) {
    return;
  }

  if (curr0 == curr1) {
    // clockwise
    ++encoderCount;
  } else {
    // anticlockwise
    --encoderCount;
  }

  return;
}

//
/// setup - runs once at power on
//

void setup() {

  // light all the individual LEDs as a test
  digitalWrite(LED_GRN, HIGH);
  digitalWrite(LED_YLW, HIGH);
  digitalWrite(STL_OUT, HIGH);

  Serial.begin (115200);
  Serial << endl << endl << F("> ** CBUS MP3 Sound Player **") << endl;

  // show code version and copyright notice
  printConfig();

  // load node variables from on-chip EEPROM
  loadNVs();

  // start the encoder and reset counters
  enc0.begin();
  enc0.reset();

  // reset display
  s7seg.reset();

  // check for module reset - if mode is SLiM and switch is depressed at startup
  pbSwitch.run();

  if (pbSwitch.isPressed() && !bFLiM) {
    // Serial << F("> switch was pressed at startup") << endl;
    doReset();
    loadNVs();
  }

  Serial << F("> mode = ") << ((bFLiM == false) ? "SLiM" : "FLiM") << F(", CANID = ") << CANID;
  Serial << F(", NN = ") << nodeNum << endl;

  if (extmem.begin(extEEPROM::twiClock400kHz) == 0) {
    // Serial << F("> using external EEPROM at I2C address 0x50 for event storage") << endl;
    EEPROM[4] = 1;
  } else {
    Serial << F("> external I2C EEPROM not found at address 0x50") << endl;
  }

  // start CAN bus and CBUS message processing
  CBUS.begin(CAN0_CS, CAN0_INT);

  // create the event hash table
  makeEvHashTable();

  // MP3 player config
  mp3Player.begin();

  // reset the switch state
  pbSwitch.reset();

  // reset the encoder counter
  enc0.reset();

  // blink node number briefly
  s7seg.displayMessage('n', lowByte(nodeNum));
  s7seg.displayNumber(0);

  // initialise the track sequence array slots to empty
  for (byte b = 0; b < MAX_SEQ_LEN; b++) {
    trackSequence[b] = 0xff;
  }

  // play the start-up track, if defined
  if (nodevars.starttrack < 0xff) {
    trackSequence[0] = nodevars.starttrack;
    rseqno = 0;
    wseqno = 1;
  }

  // set individual LEDs
  if (bFLiM == false) {
    // SLiM
    ledGrn.on();
    ledYlw.off();
  } else {
    // FLiM
    ledGrn.off();
    ledYlw.on();
  }

  ledGrn.run();
  ledYlw.run();

  // initialise sound to light
  stl.init();
  stl.setThresholdHigh(nodevars.stl_hi);
  stl.setThresholdLow(nodevars.stl_lo);

  // end of setup
  Serial << F("> ready") << endl << endl;

}

//
/// loop - runs forever
//

void loop() {

  //
  /// encoder processing
  //

  enc0.run();

  //
  /// non-blocking LED state processing
  //

  ledGrn.run();
  ledYlw.run();

  //
  /// non-blocking switch state processing
  //

  // do switch processing
  pbSwitch.run();

  //
  /// check switch hold duration for SLiM/FLiM transition, and set LED indicators
  //

  if (pbSwitch.isPressed() && pbSwitch.getCurrentStateDuration() > 6000) {

    // use LEDs to indicate that the user can release the switch
    if (!bFLiM) {
      ledYlw.blink();
      ledGrn.off();
    } else {
      ledYlw.off();
      ledGrn.on();
    }
  }

  ledGrn.run();
  ledYlw.run();

  //
  /// has switch state changed (switch held or released) ?
  //

  if (pbSwitch.stateChanged()) {

    // a double-click ?
    if (pbSwitch.isDoubleClick()) {
      doCANEnum();
    }

    // has switch been released ?
    if (!pbSwitch.isPressed()) {

      // how long was it pressed for ?
      unsigned long swTime = pbSwitch.getLastStateDuration();
      // Serial << F("> switch press duration = ") << swTime << F(" ms") << endl;

      /// long hold > 6 secs

      if (swTime > 6000) {

        // Serial << F("> long press = ") << swTime << endl;

        if (!bFLiM) {
          initFLiMProc();
        } else {
          revertSLiMProc();
        }
      }

      /// medium hold 2 - 4 secs

      if (swTime > 2000 && swTime < 4000) {

        // Serial << F("> medium press = ") << swTime << endl;

        if (bModeChanging) {
          // mode is changing from SLiM to FLiM -- cancel it and revert
          // Serial << F("> FLiM change cancelled - reverting to SLiM") << endl;
          bModeChanging = false;
          bFLiM = false;
          ledYlw.off();
          ledGrn.on();
        } else if (!bFLiM && !bModeChanging) {
          // SLiM event learning mode
          // Serial << F("> SLiM mode learning initiated") << endl;
          doSLiMLearn();
        }
      }

      /// short press 0.5 - 1.5 secs

      if (swTime > 750  && swTime < 1500) {

        // Serial << F("> short press = ") << swTime << endl;

        if (bLearn && !bFLiM) {

          // in SLiM learn mode -- short press alternates between learn and unlearn modes
          bLearnMode = !bLearnMode;
          // Serial << F("> SLiM learn mode = ") << bLearnMode << endl;

          if (bLearnMode) {
            s7seg.displayText('l', 'l');
          } else {
            s7seg.displayText('-', '-');
          }
        } else {

          // normal FLiM or SLiM mode, a short press plays or stops the currently selected track
          doPlayOrStop();
        }

      }

      ///  very short < 0.4 sec

      if (swTime < 400 && bFLiM) {
        // redo NN setup
        // not implemented for now
        // Serial << F("> very short press = ") << swTime << endl;
        initFLiMProc();
      }

    } else {
      // do any switch release processing here
    }
  }

  //
  /// check 30 sec SLiM mode learning timeout
  //

  if (bLearn == true && bFLiM == false && ((millis() - timeOutTimer) > 30000)) {
    // Serial << F("> SLiM learning timeout, timer = ") << timeOutTimer << F(", now = ") << millis() << endl;
    ledGrn.on();
    bLearn = false;
    s7seg.displayNumber(enc0.getCount());
  }

  //
  /// check 30 sec timeout for SLiM/FLiM negotiation with FCU
  //

  if (bModeChanging  && (millis() - timeOutTimer) > 30000) {
    if (bFLiM) {
      ledGrn.off();
      ledYlw.on();
    } else {
      ledGrn.on();
      ledYlw.off();
    }

    bModeChanging = false;
  }

  //
  /// encoder counter - update counter display if value has changed
  //

  if (enc0.changed()) {
    // the interrupt counter value has changed; update the display
    s7seg.displayNumber(enc0.getCount());
  }

  //
  /// check MP3 player status
  //

  if (mp3Player.available()) {

    byte ptype, pval;

    mp3Player.getStatus(&ptype, &pval);

    // a track has finished playing
    if (playing && (ptype == DFPlayerPlayFinished) == 1) {
      // ignore duplicate messages
      if (pval == lastTrackFinished && (micros() - lastCompleteTime < 200000UL)) {
        // it's a dupe message from the player - ignore it
        // Serial << F("  -- ignored duplicate player response, delta = ") << (micros() - lastCompleteTime) << endl;
      } else {
        duration = (millis() - startPlay);
        // Serial << F("> player status -- track complete, ptype = ") << ptype << endl;
        // Serial << F("> track ") << track << F(" finished, duration = ") << duration << F("ms") << endl;
        playing = false;
        startPlay = 0;
        lastTrackFinished = pval;
        lastCompleteTime = micros();
      }
    }

    // handle player errors - flash the error code on the display, e.g. E1
    if (ptype == DFPlayerError) {
      // Serial << F("> player error") << endl;

      if (pval == Busy) {
        // no card
        // Serial << F("> error - card not found") << endl;
        s7seg.displayText('n', 'c');
        playing = false;
        startPlay = 0;
        lastTrackFinished = pval;
        lastCompleteTime = micros();
      } else {
        s7seg.displayMessage('E', pval);
        s7seg.displayNumber(enc0.getCount());
        playing = false;
      }
    }

    // card removed
    if (ptype == DFPlayerCardRemoved) {
      // Serial << F("> card removed") << endl;
      s7seg.displayText('n', 'c');
      playing = false;
    }

    // card inserted
    if (ptype == DFPlayerCardInserted) {
      // Serial << F("> card inserted") << endl;
      enc0.reset();
      s7seg.displayNumber(enc0.getCount());
    }

    // printDetail(ptype, pval);
  }

  //
  /// if we're idle and not on hold, check the play sequence array for the next track/command to play/execute
  //

  if (!playing && !bOnHold) {

    byte t, rt;

    // check sequence from next slot, wrapping around from the end to the start (ring buffer)

    // count from 0 to MAX_SEQ_LEN -- calc the real offset into the buffer using the next read position
    // and correcting for ring buffer length by wrapping round to the beginning

    for (t = 0; t < MAX_SEQ_LEN; t++) {
      rt = (rseqno + t) % MAX_SEQ_LEN;
      if (trackSequence[rt] < 0xff) {
        break;
      }
    }

    if (t < MAX_SEQ_LEN) {

      // a number < 0xff was found (NB zero is a valid track number)

      // send beginning-of-sequence event if first track in sequence
      if (!bInSequence && nodevars.sendevents) {
        // Serial << F("> beginning play sequence, wpos = ") << wseqno << F(", rpos = ") << rseqno << endl;
        // Serial << F("> sending beginning-of-sequence event") << endl;
        CBUS.sendBOS(lastevtidx);
        bInSequence = true;
      }

      track = trackSequence[rt];

      // is this a track or a command ?
      if (track < PL_CMD_START) {

        // Serial << F("> playing file from sequence, slot = ") << rt << F(", track # = ") << track << F(", lag = ") << (micros() - lastCompleteTime) << "us" << endl;
        startPlay = millis();
        mp3Player.play(track);
        duration = 0;
        playing = true;
        s7seg.displayNumber(track);
        enc0.setCount(track);

      } else {
        // Serial << F("> executing command from sequence, index = ") << rt << F(", command # = ") << track << endl;
        //  send the command and pass the following EV as well - some commands need it, others will ignore it
        doPlayerCommand(track, trackSequence[rt + 1]);

        // some commands we remove from the sequence as soon as they've been executed
        if (track == PL_LOOP_MODE) {
          trackSequence[rt] = 0xff;
        }
      }

      // playing has started, so mark this location as done and reusable, unless in loop mode
      if (!bLoopMode) {
        // Serial << F("> not in loop mode, marking slot reusable") << endl;
        trackSequence[rt] = 0xff;
        if (track == PL_VOL_ABS) trackSequence[rt + 1] = 0xff;
      } else {
        // Serial << F("> in loop mode, not marking slot reusable") << endl;
      }

      // next track in the play sequence will be ...
      rseqno = (track == PL_VOL_ABS) ? rt + 2 : rt + 1;
      // Serial << F("> next slot in play sequence = ") << rseqno << endl;

    } else {
      if (bInSequence && nodevars.sendevents) {
        // Serial << F("> reached the end of play sequence, wpos = ") << wseqno << F(", rpos = ") << rseqno << endl;
        // Serial << F("> sending end-of-sequence event") << endl;
        CBUS.sendEOS(lastevtidx);
        bInSequence = false;
      }
    }
  }

  //
  /// check the 100ms CAN enumeration cycle timer
  //

  if (bCANenum && !bCANenumComplete && (millis() - CANenumTime) > 100) {

    // enumeration timer has expired -- stop enumeration and process the responses

    // Serial << F("> enum cycle complete at ") << millis() << F(", start = ") << CANenumTime << F(", duration = ") << (millis() - CANenumTime) << endl;
    // Serial << F("> processing received responses") << endl;

    for (byte i = 0; i < 16; i++) {

      // for each bit in the byte
      for (byte b = 0; b < 8; b++) {

        // ignore first bit of first byte -- CAN ID zero is not used for nodes
        if (i == 0 && b == 0) {
          continue;
        }

        // ignore if this byte is all 1's -> there are no unused IDs in this group of numbers
        if (enumResults[i] == 0xff) {
          continue;
        }

        // if the bit is not set
        if (bitRead(enumResults[i], b) == 0) {
          idChoice = ((i * 16) + b);
          // Serial << F("> bit ") << b << F(" of byte ") << i << F(" is not set, first free CAN ID = ") << idChoice << endl;
          i = 16; // ugh ... but better than a goto :)
          break;
        }
      }
    }

    // Serial << F("> responses = ") << enums << F(", lowest available ID = ") << idChoice << endl;

    CANID = idChoice;
    bCANenumComplete = true;
    bCANenum = false;
    CANenumTime = 0L;

    // store the new CAN ID in EEPROM
    EEPROM[1] = CANID;

    // send NNACK to FCU
    memset(&msg, 0, sizeof(msg));
    msg.canId = CANID;
    msg.len = 3;
    msg.rxBuf[0] = OPC_NNACK;
    msg.rxBuf[1] = highByte(nodeNum);
    msg.rxBuf[2] = lowByte(nodeNum);

    if (CBUS.sendMessage(&msg) != CAN_OK) {
      // Serial << F("> error sending NNACK") << endl;
    }
  }

  //
  /// check CAN interrupt buffer for stored frames and process the message
  //

  (void)CBUSEventProc();

  //
  /// if playing a track and the appropriate NV is set, do sound-to-light processing
  //

  if (playing && nodevars.stl) {
    stl.run();
  }

  //
  /// console user commands
  //

  if (Serial.available()) {

    byte uev = 0, vol = 0, eq = 0, fc = 0, fn = 0, eqs = 0;
    unsigned long sent = 0, rcvd = 0, errs = 0, of, ic, rt, nm;
    unsigned int np = 0;

    char c = Serial.read();

    switch (c) {

      case 'n':

        // node config
        printConfig();

        // node identity
        Serial << F("> CBUS configuration") << endl;
        Serial << F("  mode = ") << (EEPROM[0] == 0 ? "SLiM" : "FLiM") << F(", CANID = ") << EEPROM[1] << F(", NN = ") << ((EEPROM[2] << 8) + EEPROM[3]) << endl;
        Serial << endl;
        break;

      case 'e':

        // EEPROM learned event data table
        Serial << F("> Stored events ") << endl;

        Serial << F("  using ") << (nodevars.xstorage ? "external" : "onboard") << F(" EEPROM") << endl;
        sprintf(msgstr, "  max events = %d, EVs per event = %d, bytes per event = %d", EE_MAX_EVENTS, EE_NUM_EVS, EE_BYTES_PER_EVENT);
        Serial << msgstr << endl;

        for (byte j = 0; j < EE_MAX_EVENTS; j++) {
          if (getEvTableEntry(j) != 0) {
            ++uev;
          }
        }

        Serial << F("  stored events = ") << uev << F(", free = ") << (EE_MAX_EVENTS - uev) << endl;
        Serial << F("  using ") << (uev * EE_BYTES_PER_EVENT) << F(" of ") << (EE_MAX_EVENTS * EE_BYTES_PER_EVENT) << F(" bytes") << endl << endl;

        Serial << F("  Ev# |  NNhi |  NNlo |  ENhi |  ENlo  | OPC  |  ");

        for (byte j = 0; j < (EE_NUM_EVS - 1); j++) {
          sprintf(dstr, "EV%02d |  ", j + 1);
          Serial << dstr;
        }

        Serial << F("Hash |") << endl;

        Serial << F(" --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------") << endl;

        // for each event data line
        for (byte j = 0; j < EE_MAX_EVENTS; j++) {

          if (getEvTableEntry(j) != 0) {
            sprintf(dstr, "  %03d | ", j);
            Serial << dstr;

            // for each data byte of this event
            for (byte e = 0; e < (EE_NUM_EVS + 4); e++) {
              sprintf(dstr, " 0x%02hx | ", readEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + e));
              Serial << dstr;
            }

            sprintf(dstr, " %4d |", getEvTableEntry(j));
            Serial << dstr << endl;
          }
        }

        Serial << endl;

        break;

      // NVs
      case 'v':

        Serial << "> Node variables" << endl;
        Serial << F("   NV   Val") << endl;
        Serial << F("  --------------------") << endl;

        for (byte j = 0; j < EE_NUM_NVS; j++) {
          sprintf(msgstr, " - %02d : %3hd | 0x%02hx", j + 1, (byte)EEPROM[1000 + j], (byte)EEPROM[1000 + j]);
          Serial << msgstr << endl;
        }

        Serial << endl << endl;

        break;

      // CAN bus status
      case 'c':

        CBUS.getStatus(&sent, &rcvd, &errs, &of, &ic, &rt, &nm);
        Serial << F("> CAN bus status") << endl;
        Serial << F("  frames rcvd = ") << rcvd << F(", sent = ") << sent << F(", errs = ") << errs << F(", interrupts = ") << ic << F(", overruns = ") << of << endl;
        Serial << F("  buffer high water mark = ") << _messageBuffer.gethwm();
        Serial << F(", send retries = ") << rt;
        Serial << F(", no message found = ") << nm;
        Serial << F(", most recent error = ") << CBUS.getLastError() << endl;
        Serial << endl;
        break;

      // STL status
      case 's':

        Serial << F("> STL status") << endl;
        Serial << F("  NV = ") << EEPROM[EE_NVS_START + 3] << F(", curr = ") << nodevars.stl << endl;
        Serial << F("  high threshold = ") << stl.getThresholdHigh() << endl;
        Serial << F("  low threshold  = ") <<  stl.getThresholdLow() << endl;
        Serial << F("  highest reading = ") <<  stl.getHighest() << F(", lowest = ") << stl.getLowest() << F(", most recent = ") << stl.getLast() << endl;
        Serial << endl;
        break;

      // player status
      case 'm':

        mp3Player.getSettings(&vol, &eq, &fc, &fn, &np);
        Serial << F("> MP3 player status") << endl;
        Serial << F("  player volume = ") << vol << endl;
        Serial << F("  player EQ = ") << eq << endl;
        Serial << F("  player timeout = ") << EEPROM[EE_NVS_START + 2] * 10 << " ms" << endl;
        Serial << F("  current track = ") << track << " (" << fn << ")" << endl;
        Serial << F("  files on card = ") << fc << endl;
        Serial << F("  status = ") << (playing ? "playing" : "idle") << (bOnHold ? ", on hold" : "") << endl;
        Serial << F("  duration = ") << (playing ? (millis() - startPlay) : duration) << " ms" << endl;
        Serial << F("  tracks played = ") << np << endl;
        Serial << F("  track sequence -- ") << endl;

        for (byte s = 0; s < 4; s++) {
          Serial << "  | ";
          for (byte t = 0; t < 16; t++) {
            byte u = trackSequence[(s * 16) + t];
            if (u != 255)
              sprintf(dstr, " %3d | ", u);
            else
              strcpy(dstr, " ... | ");
            Serial << dstr;
          }
          Serial << endl;
        }

        Serial << "  rpos = " << rseqno << ", wpos = " << wseqno << endl;
        Serial << endl;
        break;

      case 'h':
        // event hash table
        printEvHashTable(false);
        break;

      case 'u':
        doPlayerCommand(PL_VOL_UP, 0);
        break;

      case 'd':
        doPlayerCommand(PL_VOL_DN, 0);
        break;

      case 'k':
        doPlayerCommand(PL_SKIP, 0);
        break;

      case '!':
        doPlayerCommand(PL_ESTOP, 0);
        break;

      case 'r':
        mp3Player.reset();
        break;

      case 'q':
        mp3Player.getSettings(&vol, &eq, &fc, &fn, &np);
        eqs = ((eq + 1) == 6 ? 0 : (eq + 1));
        mp3Player.setEQ(eqs);
        // Serial << F("> eq = ") << eqs << endl;
        break;

      case 'p':
        doPlayerCommand(PL_PAUSE, 0);
        break;

      case 't':
        if (!muted)
          doPlayerCommand(PL_MUTE, 0);
        else
          doPlayerCommand(PL_UNMUTE, 0);
        muted = !muted;
        break;

      case 'y':
        // reset CAN bus and CBUS message processing
        CBUS.begin(CAN0_CS, CAN0_INT);
        break;

      case '*':
        // reboot
        reboot();
        break;

      case '\r':
      case '\n':
        Serial << endl;
        break;

      default:
        // Serial << F("> unknown command ") << c << endl;
        break;
    }
  }

  //
  /// end of serial input processing
  //

  //
  /// bottom of loop()
  //
}

//
/// manually reset the module to defaults and set the module's node number
/// -- called from setup() if the button is held during power up and mode is SLiM
//

void doReset() {

  bool bDone;
  unsigned long waittime;

  // turn both LEDs off
  ledGrn.off();
  ledYlw.off();
  ledGrn.run();
  ledYlw.run();

  // start timeout timer
  waittime = millis();
  bDone = false;

  // reset the switch duration counter
  pbSwitch.resetCurrentDuration();

  // phase 1 -- wait for a further (5 sec) button press -- as a 'safety' mechanism
  while (!bDone) {

    // 30 sec timeout
    if ((millis() - waittime) > 30000) {
      ledGrn.on();
      ledYlw.off();
      return;
    }

    pbSwitch.run();

    // wait until switch held for a further 5 secs
    if (pbSwitch.isPressed() && pbSwitch.getCurrentStateDuration() > 5000) {
      bDone = true;
    }
  }

  // phase 2 -- set NN

  // show an indicator - user can release button and start NN setting

  s7seg.displayText('n', 'n');
  ledGrn.blink();
  ledYlw.blink();

  // Serial << F("> button was held for ") << pbSwitch.getCurrentStateDuration() << endl;

  // start timeout timer
  waittime = millis();
  bDone = false;

  // reset the switch state
  pbSwitch.reset();

  // process rotary encoder interrupts and update the display
  // setting is complete once button has been held for a further 5 secs
  // there is a 30 second timeout, after which control returns to setup() and thence to loop()

  // start with the current NN
  enc0.setCount(nodeNum);
  s7seg.displayNumber(enc0.getCount());

  // reset the switch duration counter -- so we don't misinterpret the switch being held down
  pbSwitch.resetCurrentDuration();

  while (!bDone) {

    // update encoder, switch and LEDs
    enc0.run();
    pbSwitch.run();
    ledGrn.run();
    ledYlw.run();

    // check timeout
    if ((millis() - waittime) > 30000) {
      ledGrn.on();
      return;
    }

    // update count and display as user operates rotary encoder
    if (enc0.changed()) {
      s7seg.displayNumber(enc0.getCount());
    }

    // wait for button press > 5 sec
    if (pbSwitch.isPressed() && pbSwitch.getCurrentStateDuration() > 5000) {
      bDone = true;
    }

  }

  // update display
  s7seg.displayText('-', '-');

  // do the reset

  // clear the entire on-chip EEPROM
  for (unsigned int j = 0; j < EEPROM.length(); j++) {
    EEPROM[j] = 0xff;
  }

  // clear the external I2C EEPROM of learned events
  resetEEPROM();

  // set the identity defaults
  EEPROM[0] = 0x00;                             // SLiM
  EEPROM[1] = lowByte(enc0.getCount());            // CAN ID
  EEPROM[2] = highByte(enc0.getCount());           // NN hi
  EEPROM[3] = lowByte(enc0.getCount());            // NN lo

  nodeNum = enc0.getCount();
  CANID = nodeNum;

  // set the NVs to default values
  EEPROM[EE_NVS_START +  0] = 5;          // volume
  EEPROM[EE_NVS_START +  1] = 0;          // eq
  EEPROM[EE_NVS_START +  2] = 50;         // comms timeout
  EEPROM[EE_NVS_START +  3] = 1;          // stl on/off
  EEPROM[EE_NVS_START +  4] = 0x02;       // stl hi/hi
  EEPROM[EE_NVS_START +  5] = 0x26;       // stl hi/lo
  EEPROM[EE_NVS_START +  6] = 0x01;       // stl lo/hi
  EEPROM[EE_NVS_START +  7] = 0xc2;       // stl lo/lo
  EEPROM[EE_NVS_START +  8] = 8;          // led display brightness
  EEPROM[EE_NVS_START +  9] = 1;          // chaining y/n
  EEPROM[EE_NVS_START + 10] = 1;          // end of sequence event y/n
  EEPROM[EE_NVS_START + 11] = 1;          // use external EEPROM y/n
  EEPROM[EE_NVS_START + 12] = 0;          // immediate play y/n
  EEPROM[EE_NVS_START + 13] = 0;          // start-up track number, 255 = none

  // reload NVs into memory
  loadNVs();

  // reset complete
  s7seg.displayNumber(nodeNum);
  ledGrn.on();
  reboot();

  return;
}

//
/// execute a play command
//

void doPlayerCommand(byte cmd, byte val) {

  byte vol = 0, eq = 0, fc = 0, fn = 0;
  unsigned int np = 0;

  // Serial << F("> executing command = ") << cmd << F(", arg = ") << val << endl;

  switch (cmd) {

    case PL_VOL_UP:
      // Serial << F("> volume up") << endl;
      mp3Player.adjustVolume(1);
      break;

    case PL_VOL_DN:
      // Serial << F("> volume down") << endl;
      mp3Player.adjustVolume(0);
      break;

    case PL_VOL_ABS:
      // Serial << F("> absolute volume level = ") << val << endl;
      mp3Player.setVolume((val > 31 ? 31 : val));
      break;

    case PL_IMMED:
      // Serial << F("> immediate play command") << endl;
      mp3Player.stop();
      bLoopMode = false;
      playing = false;
      break;

    case PL_SKIP:
      // Serial << F("> track skip") << endl;
      mp3Player.stop();
      playing = false;
      break;

    case PL_ESTOP:
      // Serial << F("> stop all") << endl;
      mp3Player.stop();
      digitalWrite(STL_OUT, LOW);
      for (byte t = 0; t < MAX_SEQ_LEN; t++) trackSequence[t] = 255;
      playing = false;
      break;

    case PL_PAUSE:
      if (playing) {
        // Serial << (paused ? "resume" : "pause") << endl;
        if (paused)
          mp3Player.start();
        else
          mp3Player.pause();
        paused = !paused;
      }

      break;

    case PL_MUTE:
      // Serial << F("> volume mute") << endl;
      mp3Player.getSettings(&vol, &eq, &fc, &fn, &np);
      lastVol = vol;
      mp3Player.setVolume(0);
      break;

    case PL_UNMUTE:
      // Serial << F("> volume unmute") << endl;
      mp3Player.setVolume(lastVol);
      break;

    case STL_ON:
      // Serial << F("> STL on") << endl;
      stl.enable(true);
      break;

    case STL_OFF:
      // Serial << F("> STL off") << endl;
      stl.enable(false);
      break;

    case PL_LOOP_MODE:
      // set loop/repeat mode
      // Serial << F("> loop mode on") << endl;
      bLoopMode = true;
      break;

    case PL_HOLD:
      // set hold mode - player won't play the sequence until unset
      bOnHold = true;
      // Serial << F("> hold mode on") << endl;
      break;

    case PL_UNHOLD:
      // unset hold mode - player can proceed with playing sequence
      bOnHold = false;
      // Serial << F("> hold mode off") << endl;
      break;

    default:
      // Serial << F("> unknown command = ") << track << endl;
      break;
  }

  return;
}

void initFLiMProc(void) {

  // Serial << F("> initiating FLiM negotation") << endl;

  ledGrn.off();
  ledYlw.blink();
  bModeChanging = true;
  timeOutTimer = millis();

  // send RQNN message with current NN, which may be zero if a virgin node
  memset(&msg, 0, sizeof(msg));
  msg.canId = CANID;
  msg.len = 3;
  msg.rxBuf[0] = OPC_RQNN;
  msg.rxBuf[1] = highByte(nodeNum);
  msg.rxBuf[2] = lowByte(nodeNum);

  // Serial << F("> requesting NN with RQNN message for NN = ") << lowByte(nodeNum) << endl;
  CBUS.sendMessage(&msg);

  return;
}

//
/// add tracks from a learned event to the play sequence
/// possibly recursively
//

byte createSequence(byte evtindex) {

  byte ev, tnum, tnext, tadd;

  tadd = 0;
  bEVChain = false;
  chainEvt = 0xff;

  // Serial << endl << F("  - creating sequence from event index = ") << evtindex << endl;

  if (evtindex >= EE_MAX_EVENTS) {
    return 0;
  }

  //  for each of 16 event vars
  for (ev = 0; ev < EE_NUM_EVS && seqadd < MAX_SEQ_LEN; ev++) {

    // the current and next EVs
    tnum = readEEPROM(EE_EVENTS_START + (evtindex * EE_BYTES_PER_EVENT) + ev + 1 + 4);
    tnext = readEEPROM(EE_EVENTS_START + (evtindex * EE_BYTES_PER_EVENT) + ev + 2 + 4);

    if (tnum == 0xff) {
      break;
    }

    // if the first and only populated EV is a command, execute it immediately and end processing
    if (ev == 0 && tnum >= PL_CMD_START && tnum != 0xff && tnext == 0xff) {
      // Serial << F("  - immediate command = ") << tnum << F(", event index = ") << evtindex << endl;
      doPlayerCommand(tnum, tnext);
      return tadd;
    }

    // if it's the chain command, record the event index number
    if (tnum == PL_EV_CHAIN) {
      bEVChain = true;
      chainEvt = tnext;
      ++ev;  // skip the next EV
      continue;
    }

    wseqno = wseqno % MAX_SEQ_LEN;

    // add this track/command to the play sequence at the next location in the sequence
    if (seqadd < MAX_SEQ_LEN) {

      if (tnum != 0xff) {
        // Serial << F("  - adding track ") << tnum << F(" at sequence index = ") << wseqno << endl;
        trackSequence[wseqno] = tnum;
        ++tadd;
        ++wseqno;
        ++seqadd;
      }

    } else {
      // Serial << F("  - reached sequence limit") << endl;
      break;
    } // can store the track

  } // for each ev

  // recursive call if chaining events
  if (bEVChain && chainEvt < EE_MAX_EVENTS) {
    // Serial << F("  - recursively chaining event index = ") << chainEvt << endl;
    tadd += createSequence(chainEvt);
    // Serial << F("  - added ") << tadd << F(" tracks") << endl;
  }

  // set the last event index used
  lastevtidx = evtindex;

  return tadd;
}

//
/// revert from FLiM to SLiM mode
//

void revertSLiMProc(void) {

  // send NNREL message
  memset(&msg, 0, sizeof(msg));
  msg.canId = CANID;
  msg.len = 3;
  msg.rxBuf[0] = OPC_NNREL;
  msg.rxBuf[1] = highByte(nodeNum);
  msg.rxBuf[2] = lowByte(nodeNum);

  CBUS.sendMessage(&msg);

  // store new mode
  EEPROM[0] = 0;
  bFLiM = EEPROM[0];

  // default CANID to same as NN
  EEPROM[1] = lowByte(nodeNum);
  CANID = EEPROM[1];

  // set indicator LEDs for SLiM mode
  ledYlw.off();
  ledGrn.on();

  return;
}

//
/// if playing, stop
/// if idle, play
//

void doPlayOrStop(void) {

  if (!playing) {
    // Serial << F("> playing track = ") << encoderCount << endl;
    track = enc0.getCount();
    mp3Player.play(track);
    stl.enable(false);
    playing = true;
    duration = 0;
    startPlay = millis();
  } else {
    // Serial << F("> stopping player") << endl;
    mp3Player.stop();
    playing = false;
    duration = 0;
    startPlay = 0;
  }

  return;
}

//
/// learn event interactively in SLiM mode
//

void doSLiMLearn(void) {

  // button was pressed for 2 secs in SLiM mode

  s7seg.displayText('l', 'l');
  bLearn = true;        // we are learning
  bLearnMode = true;    // default to learn rather than unlearn

  // start 30 sec timeout and blink the green LED
  timeOutTimer = millis();
  ledGrn.blink();

  return;
}

//
/// if in FLiM mode, initiate CAN ID enumeration cycle
//

void doCANEnum(void) {

  // due to ENUM opcode or user button press

  if (bFLiM) {

    // Serial << F("> beginning self - enumeration cycle") << endl;

    // set global variables
    bCANenum = true;                  // we are enumerating
    CANenumTime = millis();           // the cycle start time
    bCANenumComplete = false;         // the 100ms cycle has not completed
    idChoice = 1;
    enums = 0;                        // number of zero-length messages received

    // clear the results array (16 bytes * 8 bits = 128 bits)
    for (byte i = 0; i < 16; i++) {
      enumResults[i] = 0;
    }

    // send RTR frame
    memset(&msg, 0, sizeof(msg));
    msg.canId = CANID | 0x40000000;   // set RTR (remote transfer request) flag in the CAN header
    msg.len = 0;

    CBUS.sendMessage(&msg);

    // Serial << F("> enumeration cycle initiated") << endl;
  }

  return;
}

//
/// load NVs from EEPROM
//

void loadNVs(void) {

  // load NVs from EEPROM

  bFLiM =    EEPROM[0];
  CANID =    EEPROM[1];
  nodeNum = (EEPROM[2] << 8) + EEPROM[3];

  nodevars.vol =        EEPROM[EE_NVS_START + 0];
  nodevars.eq =         EEPROM[EE_NVS_START + 1];
  nodevars.timeout =    EEPROM[EE_NVS_START + 2];
  nodevars.stl =        EEPROM[EE_NVS_START + 3];
  nodevars.stl_hihi =   EEPROM[EE_NVS_START + 4];
  nodevars.stl_hilo =   EEPROM[EE_NVS_START + 5];
  nodevars.stl_lohi =   EEPROM[EE_NVS_START + 6];
  nodevars.stl_lolo =   EEPROM[EE_NVS_START + 7];
  nodevars.bright =     EEPROM[EE_NVS_START + 8];
  nodevars.chain =      EEPROM[EE_NVS_START + 9];
  nodevars.sendevents = EEPROM[EE_NVS_START + 10];
  nodevars.xstorage =   true;   // always use external EEPROM
  nodevars.immediate =  EEPROM[EE_NVS_START + 12];
  nodevars.starttrack = EEPROM[EE_NVS_START + 13];

  nodevars.stl_hi = ((unsigned int)(EEPROM[EE_NVS_START + 4] << 8) + (unsigned int)EEPROM[EE_NVS_START + 5]);
  nodevars.stl_lo = ((unsigned int)(EEPROM[EE_NVS_START + 6] << 8) + (unsigned int)EEPROM[EE_NVS_START + 7]);

  return;
}

//
/// main CBUS message processing loop
//

byte CBUSEventProc(void) {

  static byte rCANID = 0, nvindex = 0, nvval = 0, evnum = 0, evindex = 0, evval = 0;
  unsigned int nn = 0, en = 0, j = 0, opc;

  while (CBUS.available()) {

    // unsigned long mtime = micros();

    // at least one CAN frame is available in the interrupt buffer
    // retrieve the oldest one

    memset(&msg, 0, sizeof(CANFrame));
    memset(dstr, 0, sizeof(dstr));
    msg = CBUS.getNextMessage();

    // extract OPC, NN, EN, remote CANID
    opc = msg.rxBuf[0];
    nn = (msg.rxBuf[1] << 8) + msg.rxBuf[2];
    en = (msg.rxBuf[3] << 8) + msg.rxBuf[4];
    rCANID = CBUS.getCANID(msg.canId);

    // is this a CANID enumeration request from another node (RTR set) ?
    if ((msg.canId & 0x40000000) == 0x40000000) {
      // Serial << endl << F("> CANID enum RTR from CANID = ") << rCANID << endl;

      // send an empty message to show our CANID
      memset(&msg, 0, sizeof(msg));
      msg.canId = CANID;
      msg.len = 0;

      if (CBUS.sendMessage(&msg) != CAN_OK) {
        // Serial << F("> error sending enumeration response") << endl;
      }
    }

    // are we enumerating CANIDs ?
    if (bCANenum && !bCANenumComplete) {

      //  a frame with zero-length message is an ENUM response
      if (msg.len == 0) {

        // enumeratiom timer is still running -- process the CANID of this frame
        // Serial << F("> zero - length frame from CANID = ") << rCANID << endl;
        ++enums;

        // is there a clash with my current CANID ?
        if (rCANID == CANID) {
          // Serial << F("> !!! there is a clash with my CANID !!!") << endl;
        }

        // store this response in the results array
        if (rCANID > 0) {
          bitWrite(enumResults[(rCANID / 8)], rCANID % 8, 1);
          // Serial << F("> stored CANID ") << rCANID << F(" at index = ") << (rCANID / 8) << F(", bit = ") << (rCANID % 8) << endl;
        }
      }
    }

    //
    /// format and display the frame data payload
    //

    /*

        for (byte b = 0; b < 8; b++) {
          if (b < msg.len) {
            sprintf(msgstr, "0x%02hx ", msg.rxBuf[b]);
            strcat(dstr, msgstr);
          } else {
            strcat(dstr, "     ");
          }
        }

        sprintf(msgstr, "> CAN frame : CAN ID = [%3hd] len = [%1hd] OpC = [0x%02hx] data = [%s] : (%8ld)", rCANID, msg.len, opc, dstr, millis());
        Serial << msgstr << endl;
    */

    //
    /// process the message opcode
    //

    switch (opc) {

      case OPC_ACON:
      case OPC_ACOF:

        // Serial << F("> received accessory event = 0x") << _HEX(opc) << endl;

        // accessory on or off (long event with NN)
        // normally, play the tracks associated with this NN, EN and OPC
        // if SLiM and in learn mode, learn this event and associate the next free EV with the currently selected track

        // format and output CBUS message payload
        // sprintf(msgstr, "> 0x%02x : NN 0x%02hx : 0x%02hx = %4d - EN 0x%02hx : 0x%02x = %4d", opc, highByte(nn), lowByte(nn), nn, highByte(en), lowByte(en), en);
        // Serial << msgstr << endl;

        /// interactive SLiM configuration

        if (!bFLiM && bLearn) {

          // in SLiM mode and learning manually
          // learn/unlearn this event

          // Serial << F("> SLiM mode event update, nn = ") << nn << F(", en = ") << en << F(", current track = ") << encoderCount << F(", mode = ") << bLearnMode << endl;

          // search for this NN and EN pair, as we may just be adding an EV to an existing learned event
          // unlearn mode will clear the entire learned event

          // unsigned long t1 = micros();
          j = findExistingEvent(nn, en, opc);
          // Serial << F("> event lookup took ") << (micros() - t1) << "us" << endl;

          // not found and we're learning, not unlearning, so find some space for a new event
          if (j == EE_MAX_EVENTS && bLearnMode) {
            // Serial << F("> matching event not found - starting a new one") << endl;
            j = findEventSpace();
          }

          // if found or new, update the event data
          if (j < EE_MAX_EVENTS) {

            // we have found the correct event index
            byte evidx = 0;

            if (bLearnMode) {
              // store the event and track number at this location

              // Serial << F("> manually learning event at index ") << j << F(", offset = ") << (EE_EVENTS_START + (j * EE_BYTES_PER_EVENT)) << endl;

              for (evidx = 0; evidx < EE_NUM_EVS; evidx++) {
                if (readEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 5 + evidx) == 0xff) {
                  // Serial << F("> found free EV at evidx = ") << evidx << endl;
                  break;
                }
              }

              if (evidx < EE_NUM_EVS) {
                if (evidx == 0) {
                  // Serial << F("> writing event body") << endl;
                  writeEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 0, highByte(nn));
                  writeEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 1, lowByte(nn));
                  writeEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 2, highByte(en));
                  writeEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 3, lowByte(en));
                  writeEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 4, opc);
                  updateEvHashEntry(j);
                }

                // Serial << F("> writing event var at evidx = ") << evidx << endl;
                writeEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 5 + evidx, enc0.getCount());
                // Serial << F("> event learned") << endl;
              }

            } else {

              // unlearn the entire event
              cleareventEEPROM(j);
              updateEvHashEntry(j);
              // Serial << F("> manually unlearned entire event at index = ") << j << endl;
            }

            // exit learn mode
            bLearn = false;
            s7seg.displayNumber(enc0.getCount());
            ledGrn.on();
            // Serial << F("> event has been updated") << endl;

          } else {
            // failed to find existing or free storage location
            // Serial << F("> failed to update event") << endl;
          }

        } else {

          // we're in in FLiM mode, or SLiM and not learning
          // try to find a matching stored event -- match on: nn, en and opc
          // if found, capture up to 16 EVs and add as tracks to the play sequence array
          // if first and only EV is a command, just execute it and exit

          // unsigned long stm = micros();
          // unsigned long t1 = micros();

          j = findExistingEvent(nn, en, opc);
          // Serial << F("> event lookup took ") << (micros() - t1) << "us" << endl;

          if (j < EE_MAX_EVENTS) {

            // byte tadd = 0;

            // Serial << F("> found matching NN and EN at loc = ") << j << F(", processing EVs") << endl;

            // check NV to see if we chain events by default or not
            // if we don't then write to the sequence array from index 0
            // if we do, continue where we left off last time

            // Serial << F("> chain events = ") << (nodevars.chain ? "on" : "off") << endl;

            if (!nodevars.chain || nodevars.immediate) {
              // Serial << F("> (chain / immed) resetting sequence to cancel remaining playlist") << endl;

              for (wseqno = 0; wseqno < MAX_SEQ_LEN; wseqno++) {
                trackSequence[wseqno] = 0xff;
              }

              wseqno = 0;
              rseqno = 0;
            }

            if (nodevars.immediate) {
              // Serial << F("> skipping current track for immediate play of this sequence") << endl;
              doPlayerCommand(PL_SKIP, 0);
            }

            // create the sequence from the EVs in this event index, possibly recursively if the event chain command is found
            seqadd = 0;
            // Serial << F("> calling createSequence() with index = ") << j << endl;
            // tadd = createSequence(j);
            (void)createSequence(j);

            // we found and processed a matching event -- don't check any further
            // Serial << F("> lookup complete") << endl;
            // Serial << F("> added ") << tadd << F(" tracks to sequence") << endl;
            // for (j = (wseqno - tadd); j < wseqno; j++) Serial << "[" << j << "] " << trackSequence[j] << " ";
            // Serial << endl;
            // Serial << F("> sequence creation time = ") << (micros() - t1) << " us" << endl;

          } else {

            // no matching event for this nn and en
            // Serial << F("> no matching learned event found") << endl;

          } // existing ev found or not
        } // FLiM, learning or not

        break;

      case OPC_RQNP:

        // RQNP message - request for node paramters -- does not contain a NN or EN
        // Serial << F("> RQNP -- request for node params during FLiM transition for NN = ") << nn << endl;

        // only respond if we are in transition to FLiM mode
        if (bModeChanging == true) {

          // Serial << F("> we are changing mode, responding to RQNP with PARAMS") << endl;

          // respond with PARAMS message
          msg.canId = CANID;
          msg.len = 8;
          msg.rxBuf[0] = OPC_PARAMS;    // opcode
          msg.rxBuf[1] = params[1];     // manf code -- MERG
          msg.rxBuf[2] = params[2];     // minor code ver
          msg.rxBuf[3] = params[3];     // module ident -- masquerade as 6 - a CANLED (64 outputs)
          msg.rxBuf[4] = params[4];     // number of events
          msg.rxBuf[5] = params[5];     // events vars per event
          msg.rxBuf[6] = params[6];     // number of NVs
          msg.rxBuf[7] = params[7];     // major code ver

          // final param[8] = node flags is not sent here as the max message payload is 8 bytes (0-7)
          CBUS.sendMessage(&msg);

        }

        break;

      case OPC_RQNPN:

        // RQNPN message -- request parameter by index number
        // index 0 = number of params available;
        // respond with PARAN

        if (nn == nodeNum) {

          byte paran = msg.rxBuf[3];

          // Serial << F("> RQNPN request for parameter # ") << paran << endl;

          if (paran <= params[0]) {

            paran = msg.rxBuf[3];
            memset(&msg, 0, sizeof(msg));
            msg.canId = CANID;
            msg.len = 5;
            msg.rxBuf[0] = OPC_PARAN;
            msg.rxBuf[1] = highByte(nodeNum);
            msg.rxBuf[2] = lowByte(nodeNum);
            msg.rxBuf[3] = paran;
            msg.rxBuf[4] = params[paran];

            CBUS.sendMessage(&msg);

          } else {
            // Serial << F("> RQNPN - param #") << paran << F(" is out of range !") << endl;
            CBUS.sendCMDERR(9);
          }
        }

        break;

      case OPC_PARAN:

        // received PARAN -- another node responding to FCU with a parameter
        if (nn != nodeNum) {
          // not me seeing myself !
          // Serial << F("> PARAN from nn = ") << nn << F(", pn = ") << msg.rxBuf[4] << F(", pv = ") << msg.rxBuf[5] << endl;
        }

        break;

      case OPC_SNN:

        // received SNN - set node number
        if (bModeChanging == true) {

          nodeNum = (msg.rxBuf[1] << 8) + msg.rxBuf[2];
          // Serial << F("> received SNN with NN = ") << nodeNum << endl;

          // save the NN
          EEPROM[2] = msg.rxBuf[1];
          EEPROM[3] = msg.rxBuf[2];

          // set CAN ID to same as NN low byte if in the range 1-99
          if (msg.rxBuf[2] < 99 && msg.rxBuf[2] > 0) {
            EEPROM[1] = msg.rxBuf[2];
            CANID = msg.rxBuf[2];
          }

          // respond with NNACK
          msg.canId = CANID;
          msg.len = 3;
          msg.rxBuf[0] = OPC_NNACK;
          msg.rxBuf[1] = highByte(nodeNum);
          msg.rxBuf[2] = lowByte(nodeNum);

          CBUS.sendMessage(&msg);

          // we are now in FLiM mode -- set the EEPROM flag and LEDs
          EEPROM[0] = 1;
          bFLiM = true;

          ledGrn.off();
          ledYlw.on();

          bModeChanging = false;

          // Serial << F("> FLiM mode = ") << EEPROM[0] << F(", node number = ") << nodeNum << F(", CANID = ") << CANID << endl;

        }

        break;

      case OPC_NNACK:

        // received NNACK -- this is another node responding to FCU
        // Serial << F("> NNACK from nn = ") << nn << endl;

        break;

      case OPC_CANID:

        // CAN -- set CANID
        // Serial << F("> CANID for nn = ") << nn << F(" with new CANID = ") << msg.rxBuf[3] << endl;

        if (nn == nodeNum) {
          EEPROM[1] = msg.rxBuf[3];
          CANID = msg.rxBuf[3];
          // Serial << F("> setting my CANID to ") << CANID << endl;
        }

        break;

      case OPC_ENUM:

        // received ENUM -- start self-enumerate
        // Serial << F("> ENUM message for nn = ") << nn << F(" from CANID = ") << rCANID << endl;

        if (nn == nodeNum && rCANID != CANID && !bCANenum) {
          doCANEnum();
        }

        break;

      case OPC_NVRD:

        // received NVRD -- read NV by index
        nvindex = msg.rxBuf[3];
        // Serial << F("> NVRD for nn = ") << nn << F(", nv index = ") << nvindex << endl;

        if (nn == nodeNum) {

          // respond with NVANS
          msg.canId = CANID;
          msg.len = 5;
          msg.rxBuf[0] = OPC_NVANS;
          msg.rxBuf[1] = highByte(nodeNum);
          msg.rxBuf[2] = lowByte(nodeNum);
          msg.rxBuf[3] = nvindex;
          msg.rxBuf[4] = EEPROM[999 + nvindex];

          CBUS.sendMessage(&msg);
        }

        break;

      case OPC_NVSET:

        // received NVSET -- set NV by index
        nvindex = msg.rxBuf[3];
        nvval = msg.rxBuf[4];
        // Serial << F("> NVSET for index = ") << nvindex << F(", val = ") << nvval << endl;

        if (nn == nodeNum) {

          // update EEPROM for this NV -- NVs are indexed from 1, not zero
          EEPROM[EE_NVS_START + (nvindex - 1)] = nvval;

          // reload into memory
          loadNVs();

          // respond with WRACK
          CBUS.sendWRACK();
        }

        break;

      case OPC_NNLRN:

        // received NNLRN -- place into learn mode

        if (nn == nodeNum) {
          // Serial << F("> NNLRN for node = ") << nn << F(", learn mode on") << endl;
          bLearn = true;
        }

        break;

      case OPC_EVULN:

        // received EVULN -- unlearn an event, by event number
        en = (msg.rxBuf[3] << 8) + msg.rxBuf[4];
        // Serial << F("> EVULN for nn = ") << nn << F(", en = ") << en << endl;

        // we must be in learn mode
        if (bLearn == true) {

          // Serial << F("> searching for existing event to unlearn") << endl;

          // search for this NN and EN pair (ignore opcode, it isn't sent with this command)
          j = findExistingEvent(nn, en, 0xff);

          if (j < EE_MAX_EVENTS) {

            // Serial << F("> deleting event at index = ") << j << F(", evs ") << endl;
            void cleareventEEPROM();

            // update hash table
            updateEvHashEntry(j);

            // respond with WRACK
            CBUS.sendWRACK();

          } else {

            // Serial << F("> did not find event to delete") << endl;

            // respond with CMDERR
            CBUS.sendCMDERR(10);
          }

        } // if in learn mode

        break;

      case OPC_NNULN:

        // received NNULN -- exit from learn mode

        if (nn == nodeNum && bLearn == true) {
          bLearn = false;
          // Serial << F("> NNULN for node = ") << nn << F(", learn mode off") << endl;
        }

        break;

      case OPC_RQEVN:

        // received RQEVN -- request for number of stored events
        // Serial << F("> RQEVN -- number of stored events for nn = ") << nn << endl;

        if (nn == nodeNum) {

          evnum = numEvents();
          // Serial << F("> counted ") << evnum << F(" stored events") << endl;

          // respond with 0x74 NUMEV
          msg.canId = CANID;
          msg.len = 4;
          msg.rxBuf[0] = OPC_NUMEV;
          msg.rxBuf[1] = highByte(nodeNum);
          msg.rxBuf[2] = lowByte(nodeNum);
          msg.rxBuf[3] = evnum;

          CBUS.sendMessage(&msg);
        }

        break;

      case OPC_NERD:

        // received NERD 0x57 -- request for all stored events
        // Serial << F("> NERD -- request all stored events for nn = ") << nn << endl;

        if (nn == nodeNum) {

          evnum = 0;

          for (byte i = 0; i < EE_MAX_EVENTS; i++) {

            if (getEvTableEntry(i) != 0) {

              // it's a valid stored event
              // construct and send a ENRSP message

              msg.canId = CANID;
              msg.len = 8;
              msg.rxBuf[0] = OPC_ENRSP;                                                      // response opcode
              msg.rxBuf[1] = highByte(nodeNum);                                              // my NN hi
              msg.rxBuf[2] = lowByte(nodeNum);                                               // my NN lo
              msg.rxBuf[3] = readEEPROM(EE_EVENTS_START + (i * EE_BYTES_PER_EVENT) + 0);     // event NNhi
              msg.rxBuf[4] = readEEPROM(EE_EVENTS_START + (i * EE_BYTES_PER_EVENT) + 1);     // event NNlo
              msg.rxBuf[5] = readEEPROM(EE_EVENTS_START + (i * EE_BYTES_PER_EVENT) + 2);     // event ENhi
              msg.rxBuf[6] = readEEPROM(EE_EVENTS_START + (i * EE_BYTES_PER_EVENT) + 3);     // event ENlo
              msg.rxBuf[7] = i;                                                              // event table index

              CBUS.sendMessage(&msg);

            } // valid stored ev
          } // loop each ev
        } // for me

        break;

      case OPC_REVAL:

        // received REVAL -- request read of event variable by event index and ev
        // respond with NEVAL

        if (nn == nodeNum) {

          byte eventidx = msg.rxBuf[3];      // stored event index, from 0
          byte evvaridx = msg.rxBuf[4];      // event var index, from 1
          byte evval = ((byte)readEEPROM(EE_EVENTS_START + (eventidx * EE_BYTES_PER_EVENT) + 4 + (evvaridx - 1)));

          // Serial << F("> REVAL -- request event variable for nn = ") << nn << F(", en = ") << en << F(", eventidx = ") << eventidx << F(", evvaridx = ") << evvaridx << F(", evval = ") << evval << endl;

          if (getEvTableEntry(eventidx) != 0) {

            msg.canId = CANID;
            msg.len = 6;
            msg.rxBuf[0] = OPC_NEVAL;
            msg.rxBuf[1] = highByte(nodeNum);
            msg.rxBuf[2] = lowByte(nodeNum);
            msg.rxBuf[3] = eventidx;
            msg.rxBuf[4] = evvaridx;
            msg.rxBuf[5] = evval;

            CBUS.sendMessage(&msg);
          } else {

            Serial << F("> request for invalid event index") << endl;
            CBUS.sendCMDERR(6);
          }

        }

        break;

      case OPC_NNCLR:

        // received NNCLR -- clear all stored events

        if (bLearn == true && nn == nodeNum) {

          for (byte e = 0; e < EE_MAX_EVENTS; e++) {
            cleareventEEPROM(e);
          }

          // recreate the hash table
          clearEvHashTable();
          // Serial << F("> cleared all events") << endl;

          CBUS.sendWRACK();
        }

        break;

      case OPC_NEVAL:

        // this is another node responding to FCU REVAL request
        // Serial << F("> NEVAL from nn = ") << nn << endl;
        break;

      case OPC_QNN:

        // this is probably a config recreate -- respond with PNN
        msg.canId = CANID;
        msg.len = 6;
        msg.rxBuf[0] = OPC_PNN;
        msg.rxBuf[1] = highByte(nodeNum);
        msg.rxBuf[2] = lowByte(nodeNum);
        msg.rxBuf[3] = params[1];
        msg.rxBuf[4] = params[3];
        msg.rxBuf[5] = params[8];

        CBUS.sendMessage(&msg);

        break;

      case OPC_NVANS:

        // another node responding to an NVRD request
        // Serial << F("> NVANS response from node = ") << nn << endl;
        break;

      case OPC_WRACK:

        // another node sending a write ack
        // Serial << F("> WRACK response from node = ") << nn << endl;
        break;

      case OPC_RQMN:

        // request for node module name, excluding "CAN" prefix
        // Serial << F("> RQMN for nn = ") << nn << endl;

        if (nn == nodeNum) {

          // respond with NAME
          msg.canId = CANID;
          msg.len = 8;
          msg.rxBuf[0] = OPC_NAME;
          msg.rxBuf[1] = mname[0];
          msg.rxBuf[2] = mname[1];
          msg.rxBuf[3] = mname[2];
          msg.rxBuf[4] = mname[3];
          msg.rxBuf[5] = mname[4];
          msg.rxBuf[6] = mname[5];
          msg.rxBuf[7] = mname[6];

          CBUS.sendMessage(&msg);

        }

        break;

      case OPC_RQNN:

        // request for NN from another node
        // Serial << F("> RQNN from node = ") << nn << endl;
        break;

      case OPC_NUMEV:

        // a node is responding to a RQEVN request with a NUMEV reply
        break;

      case OPC_ENRSP:

        //  a node is responding to a NERD request with a ENRSP reply
        break;

      case OPC_EVLRN:

        // received EVLRN -- learn an event
        evindex = msg.rxBuf[5];
        evval = msg.rxBuf[6];

        // Serial << F("> EVLRN for source nn = ") << nn << F(", en = ") << en << F(", evindex = ") << evindex << F(", evval = ") << evval << endl;

        // if it's the first EV of the series, the value is the opcode to learn -- remember it for subsequent EVs
        if (evindex == 1) {
          // Serial << F("> first EV of the series, OpC = ") << _HEX(evval) << endl;
          lastOpc = evval;
        }

        // we must be in learn mode
        if (bLearn == true) {

          // search for this NN, EN and OPC, as we may just be adding an EV to an existing learned event
          // Serial << F("> searching for existing event to update") << endl;

          j = findExistingEvent(nn, en, lastOpc);

          // not found - it's a new event
          if (j == EE_MAX_EVENTS) {
            // Serial << F("> existing event not found - creating a new one if space available") << endl;
            j = findEventSpace();
          }

          // if existing or new event found, write the event data
          if (j < EE_MAX_EVENTS) {

            // don't actually write if EV is too large
            if (evindex <= EE_NUM_EVS) {

              // write the event to EEPROM at this location -- EVs are indexed from 1 but storage offsets start at zero !!
              // Serial << F("> writing EV = ") << evindex << F(", at index = ") << j << F(", offset = ") << (EE_EVENTS_START + (j * EE_BYTES_PER_EVENT)) << endl;

              writeEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 0, highByte(nn));
              writeEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 1, lowByte(nn));
              writeEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 2, highByte(en));
              writeEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 3, lowByte(en));
              writeEEPROM(EE_EVENTS_START + (j * EE_BYTES_PER_EVENT) + 4 + (evindex - 1), evval);
            }

            // recreate event hash table
            updateEvHashEntry(j);

            // respond with WRACK
            CBUS.sendWRACK();

          } else {

            // Serial << F("> no spare event storage, j = ") << j << endl;

            // !!! TODO  should we flash the yellow LED ??

            // respond with CMDERR
            CBUS.sendCMDERR(10);

          }

        } else { // bLearn == true
          // Serial << F("> error -- not in learn mode") << endl;
        }

        break;

      case OPC_AREQ:

        // AREQ message - request for node state
        // respond with ARON if playing or AROF if idle

        if (nn == nodeNum) {

          // Serial << F("> AREQ request for status, nn = ") << nn << F(", playing = ") << playing << F(", track = ") << track << endl;
          msg.canId = CANID;
          msg.len = 4;
          msg.rxBuf[0] = (playing ? OPC_ARON : OPC_AROF);
          msg.rxBuf[1] = highByte(nodeNum);
          msg.rxBuf[2] = lowByte(nodeNum);
          msg.rxBuf[3] = (playing ? track : 0);

          CBUS.sendMessage(&msg);

          // Serial << F("> sent response") << endl;

        }

        break;

      case OPC_BOOT:

        // boot mode -- receive a file and store in eeprom for the bootloader to find on reset
        if (nn == nodeNum) {
          // Serial << F("> received BOOT (0x5c) opcode") << endl;
          upload(&msg);
        }
        break;

      default:

        // unknown or unhandled OPC
        // Serial << F("> opcode 0x") << _HEX(opc) << F(" is not currently implemented")  << endl;

        break;
    }
  }

  // Serial << F("> end of opcode processing, time = ") << (micros() - mtime) << "us" << endl;

  //
  /// end of CBUS message processing
  //

  return 0;
}

//
/// print code version config details and copyright notice
//

void printConfig(void) {

  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << F(" for board revs. D, E & F") << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;
  Serial << F("> running on ");

#if defined(ARDUINO_AVR_UNO)
  Serial << F("ATMEGA328") << endl;
#elif defined(ARDUINO_AVR_MEGA2560)
  Serial << F("ATMEGA2560") << endl
#elif defined(ARDUINO_SAM_DUE)
  Serial << F("Arduino Due SAMD") << endl;
#else
  Serial << F("unknown processor or board") << endl;
#endif

#include <avr/io.h>

  sprintf(msgstr, "0x%02hx%02hx%02hx", SIGNATURE_0, SIGNATURE_1, SIGNATURE_2);
  Serial << F("> device signature = ") << msgstr << endl;

  // copyright
  Serial << F("> © Duncan Greenwood (MERG M5767) 2017, 2018") << endl << endl;
  return;
}

