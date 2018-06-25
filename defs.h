//
/// misc module definitions
//

#if !defined __DEFS_H__

#define __DEFS_H__

#include <Arduino.h>

const byte LED_GRN = A0;                 // green SLiM LED pin
const byte LED_YLW = A1;                 // yellow FLiM LED pin

struct _nodevars {

  byte vol;
  byte eq;
  byte timeout;
  bool stl;
  byte stl_hilo;
  byte stl_hihi;
  unsigned int stl_hi;
  byte stl_lohi;
  byte stl_lolo;
  unsigned int stl_lo;
  byte bright;
  bool chain;
  bool sendevents;
  bool xstorage;
  bool immediate;
  byte starttrack;
};

#endif

