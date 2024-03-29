/*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html      

  Copyright (c) 2020 by Hello Robot Inc.
  --------------------------------------------------------------
*/

#ifndef __WACC_H__
#define  __WACC_H__

#include "Common.h"
extern uint8_t    BOARD_VARIANT;
extern void setupWacc();         
extern void stepWaccRPC();
extern void setupBoardVariants();
void TC5_Handler();
#endif
