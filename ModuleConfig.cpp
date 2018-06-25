
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

// CBUS module configuration
//

#include <Wire.h>
#include <Streaming.h>
#include <EEPROM.h>
#include <extEEPROM.h>
#include "defs.h"
#include "ModuleConfig.h"

// extern bool useExtEEPROM;
extern extEEPROM extmem;
extern struct _nodevars nodevars;

// the event hash table
byte evthashtbl[EE_MAX_EVENTS];

//
/// lookup event, using hash table
//

byte findExistingEvent(unsigned int nn, unsigned int en, byte opc) {

  byte tarray[5];
  byte tcrc, i;

  // Serial << F("> looking for match with ") << nn << " " << en << " " << opc << endl;

  tarray[0] = highByte(nn);
  tarray[1] = lowByte(nn);
  tarray[2] = highByte(en);
  tarray[3] = lowByte(en);

  // calc hash of the incoming event to match
  tcrc = makeHash(tarray);
  // Serial << F("> event hash = ") << tcrc << endl;

  for (i = 0; i < EE_MAX_EVENTS; i++) {

    if (evthashtbl[i] == tcrc) {

      // NN + EN hash matches -- try OpC

      if (opc != 0xff) {

        if (readEEPROM(EE_EVENTS_START + (i * EE_BYTES_PER_EVENT) + 4) == opc) {
          // Serial << F("> matched event hash table at index = ") << i << endl;
          break;
        } else {
          continue;
        }

      } else {
        break;
      }
    }

  }

  // if (i == EE_MAX_EVENTS) Serial << F("> unable to find matching event") << endl;

  return i;
}

//
/// find an empty EEPROM event index -- empty locations have all locations set to 0xff / 255
//

byte findEventSpace(void) {

  byte evidx;

  for (evidx = 0; evidx < EE_MAX_EVENTS; evidx++) {
    if (evthashtbl[evidx] == 0) {
      // Serial << F("> found empty location at index = ") << evidx << endl;
      break;
    }
  }

  return evidx;
}

//
/// re/create the event hash table
//

void makeEvHashTable(void) {

  byte evarray[5];

  // Serial << F("> creating event hash table") << endl;

  for (byte i = 0; i < EE_MAX_EVENTS; i++) {

    if (readEEPROM(EE_EVENTS_START + (i * EE_BYTES_PER_EVENT)) == 0xff) {
      evthashtbl[i] = 0;
    } else {
      readEv(i, evarray);
      evthashtbl[i] = makeHash(evarray);
    }
  }

  return;
}

//
/// update a single hash table entry -- after a learn or unlearn
//

void updateEvHashEntry(byte idx) {

  byte evarray[5];
  readEv(idx, evarray);
  evthashtbl[idx] = makeHash(evarray);

  return;
}

//
/// clear the hash table
//

void clearEvHashTable(void) {

  for (byte i = 0; i < EE_MAX_EVENTS; i++) {
    evthashtbl[i] = 0;
  }

  return;
}

//
/// return an existing EEPROM event as a 4-byte array -- NN + EN
//

void readEv(byte idx, byte tarr[]) {

  // populate the first 4 bytes of the event entry from the EEPROM
  for (byte i = 0; i < EE_HASH_BYTES; i++) {
    tarr[i] = readEEPROM(EE_EVENTS_START + (idx * EE_BYTES_PER_EVENT) + i);
  }

  return;
}

//
/// print the event hash table
//

void printEvHashTable(bool raw) {

  Serial << F("> Event hash table --") << endl;

  for (byte i = 0; i < EE_MAX_EVENTS; i++) {
    if (raw)
      Serial << evthashtbl[i] << endl;
    else
      Serial << F("  -- ") << i << " - " << evthashtbl[i] << endl;
  }

  Serial << endl;
  return;
}

//
/// return number of stored events
//

byte numEvents(void) {

  byte numevents = 0;

  for (byte i = 0; i < EE_MAX_EVENTS; i++) {
    if (evthashtbl[i] > 0) {
      ++numevents;
    }
  }

  return numevents;
}

//
/// return a single hash table entry by index
//

byte getEvTableEntry(byte tindex) {


  if (tindex < EE_MAX_EVENTS) {
    return evthashtbl[tindex];
  } else {
    return 0;
  }
}

//
/// create a hash of a 4-byte event entry array -- NN + EN
//

unsigned char makeHash(byte tarr[]) {

  unsigned char hash;
  unsigned int nn, en;

  // make a hash of a 4-byte NN + EN event

  nn = (tarr[0] << 8) + tarr[1];
  en = (tarr[2] << 8) + tarr[3];

  // need to hash the NN and EN to a uniform distribution across HASH_LENGTH
  hash = nn ^ (nn >> 8);
  hash = 7 * hash + (en ^ (en >> 8));

  // ensure it is within bounds of eventChains
  hash %= HASH_LENGTH;

  return hash;
}

//
/// read an NV value from local EEPROM
//

byte readNv(byte idx) {

  return (EEPROM[EE_NVS_START + idx]);
}

//
/// write an NV value to local EEPROM
//

void writeNv(byte idx, byte val) {

  EEPROM[EE_NVS_START + idx] = val;
  return;
}

//
/// read a byte from EEPROM
//

byte readEEPROM(unsigned int eeaddress) {

  byte rdata = 0xFF;
  byte r;

  // Serial << F(" -- read, addr = ") << eeaddress << endl;

  pinMode(SCL, OUTPUT);
  r = extmem.begin(extEEPROM::twiClock400kHz);

  if (nodevars.xstorage) {
    r = extmem.read(eeaddress, &rdata, 1);

    if (r != 0) {
      // Serial << F("> I2C write error = ") << r << endl;
    }

    return rdata;
  } else {
    return (EEPROM[eeaddress]);
  }
}

//
/// write a byte
//

void writeEEPROM(unsigned int eeaddress, byte data) {

  byte r;

  // Serial << F(" -- write, addr = ") << eeaddress << F(", data = ") << data << endl;

  pinMode(SCL, OUTPUT);
  r = extmem.begin(extEEPROM::twiClock400kHz);

  if (nodevars.xstorage) {

    r = extmem.write(eeaddress, &data, 1);

    if (r != 0) {
      // Serial << F("> I2C write error = ") << r << endl;
    }

  } else {
    EEPROM[eeaddress] = data;
  }

  return;
}

//
/// write an entire event
//

void writeeventEEPROM(byte index, byte data[]) {

  // the array should contain EE_BYTES_PER_EVENT (21) bytes -- NN x2, EN x2, OpC, EE_NUM_EVS

  byte r;
  int eeaddress = EE_EVENTS_START + (index * EE_BYTES_PER_EVENT);

  if (nodevars.xstorage) {

    pinMode(SCL, OUTPUT);
    r = extmem.begin(extEEPROM::twiClock400kHz);

    // Serial << F(" -- write event, index = ") << index << F(", addr = ") << eeaddress << endl;
    r = extmem.write(eeaddress, data, EE_BYTES_PER_EVENT);

    if (r != 0) {
      Serial << F("> I2C write error = ") << r << endl;
    }

  } else {

    for (byte i = 0; i < EE_BYTES_PER_EVENT; i++) {
      EEPROM[eeaddress + i] = data[i];
    }

  }

  return;
}

//
/// clear an event from the table
//

void cleareventEEPROM(byte index) {

  byte tarray[EE_BYTES_PER_EVENT];
  byte i;

  // create an array of 0xff
  for (i = 0; i < EE_BYTES_PER_EVENT; i++) {
    tarray[i] = 0xff;
  }

  // Serial << F("> clearing event at index = ") << index << endl;
  writeeventEEPROM(index, tarray);

  return;
}

//
/// clear all event data in EEPROM
//

void resetEEPROM(void) {

  // Serial << F("> in resetEEPROM()") << endl;

  int j;

  // clear first 8K bytes (64 kbits) of external to 0xff
  for (j = 0; j < 8192; j++) {
    writeEEPROM(j, 0xff);
  }

  return;
}

