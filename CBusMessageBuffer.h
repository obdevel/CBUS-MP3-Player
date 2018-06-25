
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

#ifndef _MSGBUFFER_H_
#define __MSGBUFFER_H_

#include <Arduino.h>

// CBUS message buffer size - max number of entries
#define CBUSMSGBUFFERSIZE 20

typedef struct {
  unsigned long canId;
  byte len;
  unsigned char rxBuf[8];
} CANFrame;

class cbusMessageBuffer {

  private:
    volatile byte _writePointer;
    volatile byte _readPointer;
    volatile unsigned long _count;
    volatile unsigned long _overflows;
    volatile unsigned long _hwm;

    volatile struct {
      CANFrame messageRecord;
    } _messageBuffer[CBUSMSGBUFFERSIZE];

  public:
    cbusMessageBuffer();
    bool addMessage (CANFrame *newMessage);
    CANFrame getMessage(void);
    bool available(void);
    byte gethwm(void);
    void resethwm();

};

#endif
