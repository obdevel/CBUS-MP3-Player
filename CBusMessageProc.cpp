
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
#include <Streaming.h>
#include <mcp_can.h>
#include "defs.h"
#include "CBusMessageBuffer.h"
#include "CBusMessageProc.h"
#include "DisplayProc.h"
#include "cbusdefs.h"

#define MAX_SEND_ATTEMPTS 10

extern unsigned int nodeNum;
extern byte CANID;
extern MCP_CAN _CAN0;
extern cbusMessageBuffer _messageBuffer;
extern struct _nodevars nodevars;


CANBus::CANBus() {

  // constructor
};

void CANBus::begin(byte CSpin, byte INTpin) {

  //
  /// initialise the CAN controller and attach ISR
  //

  int ret;

  // set the appropriate mode for the interrupt and CS pins
  pinMode(INTpin, INPUT);
  pinMode(CSpin, OUTPUT);

  this->_numMsgsSent = 0;
  this->_numMsgsRcvd = 0;
  this->_numErrs = 0;
  this->_retries = 0;
  this->_interruptCount = 0;
  this->_overflowCount = 0;
  this->_errframes = 0;
  this->_lastErr = 0;
  this->_nomsgs = 0;
  _messageBuffer.resethwm();

  // init controller
  if ((ret = _CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_16MHZ)) == CAN_OK) {
    // Serial << F("> MCP2515 SPI CAN controller initialised successfully") << endl;
  } else {
    Serial << F("> error initialising MCP2515, err = ") << ret << endl;
  }

  // set operation mode to normal so the MCP2515 sends acks to received data.
  _CAN0.setMode(MCP_NORMAL);

  // attach MCP2515 CAN controller interrupt
  attachInterrupt(digitalPinToInterrupt(INTpin), bufferMessage, FALLING);

  return;
}

//
/// check for unprocessed messages in the buffer
//

bool CANBus::available(void) {

  return (_messageBuffer.available());
}

//
/// get oldest unprocessed message from the buffer
//

CANFrame CANBus::getNextMessage(void) {

  this->_msg = _messageBuffer.getMessage();
  this->_numMsgsRcvd++;
  return this->_msg;
}

//
/// send a CBUS message
//

/*

  -- from can_defs.h

  #define CAN_OK             (0)
  #define CAN_FAILINIT       (1)
  #define CAN_FAILTX         (2)
  #define CAN_MSGAVAIL       (3)
  #define CAN_NOMSG          (4)
  #define CAN_CTRLERROR      (5)
  #define CAN_GETTXBFTIMEOUT (6)
  #define CAN_SENDMSGTIMEOUT (7)
  #define CAN_FAIL           (0xff)

*/

//
/// send a CAN bus frame
//

byte CANBus::sendMessage(CANFrame *msg) {

  // caller must populate the frame data
  // we set the priority in the header

  byte _res, _attempts;

  // set the CBUS message priority
  bitSet(msg->canId, 7);
  bitSet(msg->canId, 8);
  bitClear(msg->canId, 9);
  bitSet(msg->canId, 10);

  for (_attempts = 0; _attempts < MAX_SEND_ATTEMPTS; _attempts++) {

    // Serial << F("> CBUS message send attempt = ") << _attempts << endl;

    if ((_res = _CAN0.sendMsgBuf(msg->canId, msg->len, msg->rxBuf)) == CAN_OK) {

      // Serial << F("> sent CAN frame for CAN ID = ") << getCANID(msg->canId) << endl;
      this->_numMsgsSent++;
      break;

    } else {
      Serial << F("> error sending CAN frame, msg # = ") << _numMsgsSent << F(", err = ") << _res << F(", attempt = ") << _attempts << endl;
      delay(10);
    }
  }

  this->_retries += _attempts;
  delay(_attempts * 10);

  if (_attempts == MAX_SEND_ATTEMPTS) {
    this->_numErrs++;
    this->_lastErr = _res;
  }

  return _res;
}

//
/// extract CANID from CAN frame header
//

byte CANBus::getCANID(unsigned long header) {

  return (header & 0x7f);
}

byte CANBus::sendWRACK(void) {

  // send a write acknowledgement response
  byte rv;
  CANFrame msg;

  memset(&msg, 0, sizeof(msg));
  msg.canId = CANID;
  msg.len = 3;
  msg.rxBuf[0] = OPC_WRACK;
  msg.rxBuf[1] = highByte(nodeNum);
  msg.rxBuf[2] = lowByte(nodeNum);

  rv = this->sendMessage(&msg);
  return rv;
}

byte CANBus::sendCMDERR(byte cerrno) {

  // send a command error response
  byte rv;
  CANFrame msg;

  memset(&msg, 0, sizeof(msg));
  msg.canId = CANID;
  msg.len = 4;
  msg.rxBuf[0] = OPC_CMDERR;
  msg.rxBuf[1] = highByte(nodeNum);
  msg.rxBuf[2] = lowByte(nodeNum);
  msg.rxBuf[3] = cerrno;

  rv = this->sendMessage(&msg);
  return rv;
}

byte CANBus::sendBOS(byte evtidx) {

  // send beginning-of-sequence event
  byte rv = 0;
  CANFrame msg;

  if (nodevars.sendevents) {

    memset(&msg, 0, sizeof(msg));
    msg.canId = CANID;
    msg.len = 5;
    msg.rxBuf[0] = OPC_ACON;
    msg.rxBuf[1] = highByte(nodeNum);
    msg.rxBuf[2] = lowByte(nodeNum);
    msg.rxBuf[3] = 0;
    msg.rxBuf[4] = 0;

    rv = this->sendMessage(&msg);
  }

  return rv;
}

byte CANBus::sendEOS(byte evtidx) {

  // send end-of-sequence event
  byte rv = 0;
  CANFrame msg;

  if (nodevars.sendevents) {

    memset(&msg, 0, sizeof(msg));
    msg.canId = CANID;
    msg.len = 5;
    msg.rxBuf[0] = OPC_ACOF;
    msg.rxBuf[1] = highByte(nodeNum);
    msg.rxBuf[2] = lowByte(nodeNum);
    msg.rxBuf[3] = 0;
    msg.rxBuf[4] = 0;

    rv = this->sendMessage(&msg);
  }

  return rv;
}

void CANBus::getStatus(unsigned long *sent, unsigned long *rcvd, unsigned long *errs, unsigned long *of, unsigned long *ic, unsigned long *rt, unsigned long *nm) {

  *sent = this->_numMsgsSent;
  *rcvd = this->_numMsgsRcvd;
  *errs = this->_numErrs;
  *of = this->_overflowCount;
  *ic = this->_interruptCount;
  *rt = this->_retries;
  *nm = this->_nomsgs;

  return;
}

byte CANBus::getLastError(void) {

  return this->_lastErr;
}

void CANBus::setLastError(byte err) {

  this->_lastErr = err;
  return;
}

void CANBus::incrOverflow(void) {

  this->_overflowCount++;
  return;
}

void CANBus::incrInterrupts(void) {

  this->_interruptCount++;
  return;
}

void CANBus::incrNoMsgs(void) {
  this->_nomsgs++;
  return;
}

//
/// external interrupt service routine for MCP2515 CAN controller
/// get a CAN message and add it to the receive ring buffer
//

extern CANBus CBUS;

void bufferMessage(void) {

  CANFrame msg;

  memset(&msg, 0, sizeof(msg));

  if (_CAN0.checkReceive() == CAN_MSGAVAIL) {

    if (_CAN0.readMsgBuf(&msg.canId, &msg.len, msg.rxBuf) != CAN_OK) {     // returns CAN_NOMSG or CAN_OK
      // no message waiting
      CBUS.incrNoMsgs();

    } else if (_CAN0.checkError() != CAN_OK) {    // 0-7 and 0xff, CAN_OK == 0
      // flag error - don't buffer a message
      CBUS.setLastError(_CAN0.getError());

    } else {

      // buffer the incoming message
      if (!(_messageBuffer.addMessage(&msg))) {
        CBUS.incrOverflow();
      }

      // CBUS.setLastError(0);
      CBUS.incrInterrupts();
    }
  }

  return;
}

