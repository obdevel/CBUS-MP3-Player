
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
/// class to implement a CAN bus receive buffer
//

#include "defs.h"
#include "CBusMessageBuffer.h"

// constructor

cbusMessageBuffer::cbusMessageBuffer() {

  _writePointer = 0;
  _readPointer = 0;
  _count = 0;
  _hwm = 0;
  _messageBuffer[_writePointer].messageRecord.canId = 0;
  _messageBuffer[_writePointer].messageRecord.len = 0;
}

// add a received frame to the buffer
// called from the MCP2515 interrupt handler

bool cbusMessageBuffer::addMessage (CANFrame *newMessage) {

  if (_count == CBUSMSGBUFFERSIZE) {
    // buffer is full -- drop the frame & return failure
    return false;
  }

  // store the new message at the next slot
  _messageBuffer[_writePointer].messageRecord.canId = newMessage->canId;
  _messageBuffer[_writePointer].messageRecord.len = newMessage->len;
  memcpy((void *) & (_messageBuffer[_writePointer].messageRecord.rxBuf), (void *)(newMessage->rxBuf), newMessage->len * sizeof(unsigned char));

  // increment write pointer and wrap if necessary
  _writePointer = ((_writePointer + 1) % CBUSMSGBUFFERSIZE);

  // increment counter & highwater mark;
  _count++;
  _hwm = (_count > _hwm ? _count : _hwm);

  return true;
}

// return the next frame in the interrupt buffer
// should only be called if .available() is true

CANFrame cbusMessageBuffer::getMessage() {

  CANFrame nextMessage;

  noInterrupts();
  nextMessage.canId = _messageBuffer[_readPointer].messageRecord.canId;
  nextMessage.len   = _messageBuffer[_readPointer].messageRecord.len;
  memcpy((void *) & (nextMessage.rxBuf), (void *) & (_messageBuffer[_readPointer].messageRecord.rxBuf), 8 * sizeof(unsigned char));
  _readPointer = ((_readPointer + 1) % CBUSMSGBUFFERSIZE);
  interrupts();
  _count--;

  return nextMessage;
}

// are there any buffered frames to read ?

bool cbusMessageBuffer::available(void) {

  return (_count > 0);
}

byte cbusMessageBuffer::gethwm(void) {

  return _hwm;
}

void cbusMessageBuffer::resethwm(void) {

  _hwm = 0;
  return;
}


