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

#define LIFT_MOTOR 1
#define OMNI_0_MOTOR 2
#define OMNI_1_MOTOR 3
#define OMNI_2_MOTOR 4
#define ARM_MOTOR 5
#define EOA_MOTOR 6

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
