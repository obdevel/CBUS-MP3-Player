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
/// functions related to uploading new code over CAN and rebooting
//

#include <avr/wdt.h>
#include <Streaming.h>
#include <extEEPROM.h>

#include "defs.h"
#include "CBUSMessageBUffer.h"
#include "CBUSMessageProc.h"
#include "ModuleConfig.h"
#include "cbusdefs.h"
#include "bootload.h"

extern CANBus CBUS;
extern CANFrame msg;
extern unsigned int nodeNum;
extern extEEPROM extmem;
extern char msgstr[100], dstr[50];

//
/// receive a new program data file by CAN and store in i2C EEPROM
//

void upload(CANFrame *imsg) {

  bool bComplete = false, bSuccess = false, bReboot;
  byte rv;
  unsigned int nframes = 0;
  unsigned long addrOffset = 2752;  // next multiple of 64 byte eeprom pagesize (128 events of 21 bytes each consume the first 2688 bytes of external EEPROM)
  unsigned long lastframe = millis();
  unsigned long xbytes, tbytes = 0UL;


  Serial << F("> starting boot file upload transaction") << endl;

  // read the initial BOOT message forwarded here
  xbytes = (imsg->rxBuf[3] * 16777216UL) + (imsg->rxBuf[4] * 65336UL) + (imsg->rxBuf[5] * 256UL) + imsg->rxBuf[6];
  bReboot = imsg->rxBuf[7];

  if (bReboot) {
    Serial << F("> reboot after upload requested") << endl;
  }

  Serial << F("> expected filesize = ") << xbytes << endl;

  // reset the bootloader control byte to zero = no action to take on boot
  if (extmem.write(2750UL, 0) != 0) {
    Serial << F("> error writing control byte") << endl;
  }

  // initial ack
  Serial << F("> sending WRACK") << endl;
  CBUS.sendWRACK();
  while (!bComplete) {

    if (CBUS.available()) {

      // flash the green LED
      if (nframes % 10 == 0) {
        digitalWrite(LED_GRN, !digitalRead(LED_GRN));
      }

      memset(&msg, 0, sizeof(CANFrame));
      msg = CBUS.getNextMessage();

      // data comes in extended frames -- 29 bit CAN ID plus 8 bytes of data
      if ((msg.canId & 0x80000000) == 0x80000000) {

        if (msg.len == 0) {
          // end of transaction
          Serial << F("> end of transaction") << endl;
          bSuccess = true;
          bComplete = true;
          continue;
        }

        ++nframes;

        // write the message payload to EEPROM, and allow delay for EEPROM to complete the transaction
        rv = extmem.write(addrOffset, msg.rxBuf, msg.len);
        delay(6);

        if (rv == 0) {
          addrOffset += msg.len;
          tbytes += msg.len;

          // send ack message
          CBUS.sendWRACK();

          if (tbytes % 1024 == 0) {
            Serial << F("> recvd/wrote ") << tbytes << F(" bytes") << endl;
          }

        } else {

          Serial << F("> eeprom write error = ") << rv << F(" at addr = ") << addrOffset << endl;

          if (rv == EEPROM_ADDR_ERR)
            Serial << F("> no space left on device") << endl;

          //  send error message and exit loop
          CBUS.sendCMDERR(1);
          bSuccess = false;
          bComplete = true;
          continue;
        }

        // ext frame ??

      } else {
        Serial << F("> ignoring standard frame, opcode = ") << msg.rxBuf[0] << endl;
      }

      lastframe = millis();

      // CAN available ??
    }

    // 10 second timeout
    if ((millis() - lastframe) > 10000) {
      Serial << F("> bus receive timeout") << endl;
      bComplete = true;
    }

    // complete ??
  }

  Serial << endl;

  Serial << F("> recvd/wrote ") << tbytes << F(" bytes in ") << nframes << F(" frames") << endl;
  Serial << F("> end of upload transaction") << endl;

  if (bSuccess) {

    // final ack
    CBUS.sendWRACK();

    // write status, signature and filesize to EEPROM for bootloader to read
    if (extmem.write(2750UL, 1) != 0) {
      Serial << F("> error writing control byte") << endl;
    }

    byte ldata[4];
    ldata[0] = 'B';
    ldata[1] = 'O';
    ldata[2] = 'O';
    ldata[3] = 'T';

    if ((extmem.write(2755UL, ldata, sizeof(ldata))) != 0) {
      Serial << F("> error writing signature") << endl;
    }

    ldata[0] = (byte)(tbytes >> 24);
    ldata[1] = (byte)(tbytes >> 16);
    ldata[2] = (byte)(tbytes >> 8);
    ldata[3] = (byte)(tbytes);

    if ((extmem.write(2751UL, ldata, sizeof(ldata))) != 0) {
      Serial << F("> error writing filesize") << endl;
    }

    // Serial << F("> bootloader control byte = ") << extmem.read(2750) << endl;

    // extmem.read(2751UL, ldata, 4);
    // tbytes = (ldata[0] * 16777216UL) + (ldata[1] * 65336UL) + (ldata[2] * 256UL) + ldata[3];
    // Serial << F("> filesize = ") << tbytes << endl;

    if (bReboot) {
      Serial << F("> successful upload ... rebooting ...") << endl;
      Serial.flush();
      reboot();
    } else {
      Serial << F("> reboot deferred ...") << endl;
    }
  } else {
    Serial << F("> upload failed ... aborting") << endl << endl;
  }

  Serial << endl;
  return;
}

//
/// reboot the MCU by setting the watchdog timer and allowing it to expire
//

void reboot() {

  wdt_enable(WDTO_15MS);
  noInterrupts();

  // set the "external reset" flag so the bootloader runs on restart
  while (true)
    MCUSR |= _BV(EXTRF);

}

