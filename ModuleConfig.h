
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

// ModuleConfig.h

#if !defined _CBUSMODULECONFIG_
#define _CBUSMODULECONFIG_

#include <Arduino.h>
#include <Streaming.h>

// definitions for EEPROM event storage
// -- on-chip storage is 72 events of 8 tracks
// -- external storage is 128 events of 16 tracks

// on-chip EEPROM
// #define EE_EVENTS_START 10
// #define EE_MAX_EVENTS 72
// #define EE_NUM_EVS (8 + 1)                      // 9
// #define EE_BYTES_PER_EVENT (4 + EE_NUM_EVS)     // 13

// external EEPROM
#define EE_EVENTS_START 10
#define EE_MAX_EVENTS 128
#define EE_NUM_EVS (16 + 1)                        // 17
#define EE_BYTES_PER_EVENT (4 + EE_NUM_EVS)        // 21

// EVs
#define EE_NVS_START 1000
#define EE_NUM_NVS 16

// hash table
#define EE_HASH_BYTES 4
#define HASH_LENGTH 128

byte findExistingEvent(unsigned int nn, unsigned int en, byte opc);
byte findEventSpace(void);

void getEvArray(byte idx);
void makeEvHashTable(void);
void updateEvHashEntry(byte idx);
void readEv(byte idx, byte tarr[]);
void clearEvHashTable(void);
void printEvHashTable(bool raw);
byte getEvTableEntry(byte tindex);
byte numEvents(void);

unsigned char makeHash(byte tarr[]);

byte readNv(byte idx);
void writeNv(byte idx, byte val);

byte readEEPROM(unsigned int eeaddress);
void writeEEPROM(unsigned int eeaddress, byte data);
void writeeventEEPROM(byte index, byte data[]);
void cleareventEEPROM(byte index);
void resetEEPROM(void);

#endif

