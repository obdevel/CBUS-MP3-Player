
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

#if !defined _MP3PLAYERPROC_
#define _MP3PLAYERPROC_

#include <SoftwareSerial.h>
#include <Streaming.h>
#include <DFRobotDFPlayerMini.h>

class MP3Player {

  public:
    MP3Player(void);
    void begin(void);
    bool available(void);
    void getStatus(byte *ptype, byte *pval);
    void getSettings(byte *vol, byte *eq, byte *fc, byte *fn, unsigned int *nump);
    void play(byte track);
    void stop(void);
    void pause(void);
    void start(void);
    void setVolume(byte vol);
    void setEQ(byte eq);
    void adjustVolume(bool direction);
    void printDetails(byte type, byte value);
    void reset(void);

  private:
    unsigned int _numPlayed;
};

#endif

