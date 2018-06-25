
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
//
//

#include "defs.h"
#include "MP3PlayerProc.h"

/*
  - from library header:

  #define DFPLAYER_EQ_NORMAL 0
  #define DFPLAYER_EQ_POP 1
  #define DFPLAYER_EQ_ROCK 2
  #define DFPLAYER_EQ_JAZZ 3
  #define DFPLAYER_EQ_CLASSIC 4
  #define DFPLAYER_EQ_BASS 5
*/

#define SS_RX 6                    // serial from MP3 player
#define SS_TX 7                    // serial to MP3 player

DFRobotDFPlayerMini miniplayer;
SoftwareSerial mp3Serial(SS_RX, SS_TX);
extern struct _nodevars nodevars;


MP3Player::MP3Player() {

  // constructor
  this->_numPlayed = 0;
}

void MP3Player::begin(void) {

  pinMode(SS_TX, OUTPUT);
  pinMode(SS_RX, INPUT);

  mp3Serial.begin(9600);

  if (miniplayer.begin(mp3Serial, true, false)) {
    // Serial << F("> MP3 player initialised") << endl;
  } else {
    Serial << F("> error initialising MP3 player: please insert a valid memory card") << endl;
  }

  delay(250);
  miniplayer.setTimeOut(nodevars.timeout * 10);
  delay(50);
  miniplayer.outputDevice(DFPLAYER_DEVICE_SD);
  delay(50);

  // player bug at power on -- reduce to zero step by step
  for (byte v = 0; v < 32; v++) {
    miniplayer.volumeDown();
  }

  miniplayer.volume(nodevars.vol);
  delay(50);
  miniplayer.EQ(nodevars.eq);
  delay(50);
  miniplayer.outputSetting(true, 15);
  delay(50);
  miniplayer.disableLoop();
  delay(50);
  miniplayer.enableDAC();

  return;
}

bool MP3Player::available(void) {

  return (miniplayer.available());
}

void MP3Player::getStatus(byte *ptype, byte *pval) {

  delay(50);
  *ptype = miniplayer.readType();
  delay(50);
  *pval = miniplayer.read();

  return;
}

void MP3Player::getSettings(byte *vol, byte *eq, byte *fc, byte *fn, unsigned int *nump) {

  *vol = miniplayer.readVolume();
  delay(50);
  *eq = miniplayer.readEQ();
  delay(50);
  *fc = miniplayer.readFileCounts();
  delay(50);
  *fn = miniplayer.readCurrentFileNumber();
  *nump = this->_numPlayed;

  return;
}

void MP3Player::play(byte track) {

  miniplayer.playMp3Folder(track);
  this->_numPlayed++;
  return;
}

void MP3Player::stop(void) {

  miniplayer.stop();
  return;
}

void MP3Player::pause(void) {

  miniplayer.pause();
  return;
}

void MP3Player::start() {

  miniplayer.start();
  return;
}

void MP3Player::setVolume(byte vol) {

  miniplayer.volume(vol);
  return;
}

void MP3Player::setEQ(byte eq) {

  miniplayer.EQ(eq);
  return;
}

void MP3Player::adjustVolume(bool direction) {

  if (direction) miniplayer.volumeUp();
  else miniplayer.volumeDown();

  return;
}

void MP3Player::reset(void) {

  miniplayer.reset();
  return;
}

void MP3Player::printDetails(byte type, byte value) {

  /* from library header file --

    #define TimeOut 0
    #define WrongStack 1
    #define DFPlayerCardInserted 2
    #define DFPlayerCardRemoved 3
    #define DFPlayerCardOnline 4
    #define DFPlayerPlayFinished 5
    #define DFPlayerError 6

    #define Busy 1
    #define Sleeping 2
    #define SerialWrongStack 3
    #define CheckSumNotMatch 4
    #define FileIndexOut 5
    #define FileMismatch 6
    #define Advertise 7

  */

  switch (type) {
    case TimeOut:
      Serial << F("  -- time out");
      break;
    case WrongStack:
      Serial << F("  -- stack error");
      break;
    case DFPlayerCardInserted:
      Serial << F("  -- card inserted");
      break;
    case DFPlayerCardRemoved:
      Serial << F("  -- card removed");
      break;
    case DFPlayerCardOnline:
      Serial << F("  -- card online");
      break;
    case DFPlayerPlayFinished:
      Serial << F("  -- track = ") << value << F(" complete (") << micros() << F(")");
      break;
    case DFPlayerError:
      Serial << F("  -- player error =  ") << value << F(", - ");

      switch (value) {
        case Busy:
          Serial << F("card not found");
          break;
        case Sleeping:
          Serial << F("sleeping");
          break;
        case SerialWrongStack:
          Serial << F("stack error");
          break;
        case CheckSumNotMatch:
          Serial << F("bad checksum");
          break;
        case FileIndexOut:
          Serial << F("file index out of range");
          break;
        case FileMismatch:
          Serial << F("cannot find file");
          break;
        case Advertise:
          Serial << F("in ad mode");
          break;
        default:
          break;
      }

      break;

    default:
      Serial << F("  -- unknown status = ") << type << F(", ") << value;
      break;
  }

  Serial << F(" - ") << type << F(", ") << value << endl;

  return;
}


