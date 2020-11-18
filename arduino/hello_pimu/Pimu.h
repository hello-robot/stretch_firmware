/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __PIMU_H__
#define  __PIMU_H__

#include "Common.h"


extern void setupPimu();         
extern void stepPimuRPC();
extern void toggle_led(int rate_ms);
void TC5_Handler();

extern void on_runstop();
extern void do_beep(int bid);


#endif
