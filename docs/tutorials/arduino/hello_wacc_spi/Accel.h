/*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc Spi

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html  

  Copyright (c) 2020 by Hello Robot Inc.
  --------------------------------------------------------------
*/

#ifndef __ACCEL_H__
#define  __ACCEL_H__
#include "Common.h"

extern float ax,ay,az;
extern uint32_t single_tap_count;
extern uint32_t double_tap_count;

void setupAccel();
void stepAccel();         
void onAccelInt1();
void onAccelInt2();
void setAccelRange(uint8_t r);
void setSingleTapSensitivity(uint8_t dur, uint8_t thresh);
extern uint32_t accel_debug;

#endif
