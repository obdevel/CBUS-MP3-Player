
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

#if !defined _STLPROC_
#define _STLPROC_

#include <Arduino.h>

class STL {

  public:
    STL(byte inpin, byte outpin);
    void init(void);
    void run(void);
    void enable(bool state);

    void setThresholdHigh(unsigned int level);
    void setThresholdLow(unsigned int level);

    unsigned int getThresholdHigh(void);
    unsigned int getThresholdLow(void);
    unsigned int getHighest(void);
    unsigned int getLowest(void);
    unsigned int getLast(void);
    bool getState(void);

  private:
    byte _inpin, _outpin, _state;
    unsigned int _threshhi, _threshlo, _high, _low, _val;

};

#endif

