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

extern uint8_t    BOARD_VARIANT;
extern uint8_t    BOARD_VARIANT_DEDICATED_SYNC;

extern void setupPimu();         
extern void setupADC();
extern void stepPimuRPC();
extern void setupBoardVariants();

extern void toggle_led(int rate_ms);


extern void on_runstop();
extern void do_beep(int bid);


#endif
