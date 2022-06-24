/*
  -------------------------------------------------------------
  Hello Robot - Hello Stepper

  This code is derived from the Mechaduino project. 
  https://github.com/jcchurch13/Mechaduino-Firmware

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc.
  --------------------------------------------------------------
*/

#ifndef __HELLO_CONTROLLER_H__
#define  __HELLO_CONTROLLER_H__

#include "Common.h"




extern volatile bool hello_interface;

extern void setupHelloController();
extern void stepHelloController();
extern void stepHelloCommutation();
extern void stepHelloControllerRPC();
extern void toggle_led(int rate_ms);
extern float debug;
extern void setupMGInterrupts();
extern void enableMGInterrupts();
extern void disableMGInterrupts(); 
extern float FsCtrl;
extern Status stat, stat_out;

extern uint8_t    BOARD_VARIANT;
extern uint8_t    BOARD_VARIANT_DRV8842;
extern uint8_t    BOARD_VARIANT_PIN_RUNSTOP;
extern void setupBoardVariants();

#define TC4_LOOP_RATE 1000                                             //Update rate of control loop Hz
#define TC4_COUNT_PER_CYCLE (int)( round(48000000 / 2/ TC4_LOOP_RATE))  //24,000 at 1Khz, 2:1 prescalar TC4 is 32bit timer 
#define US_PER_TC4_CYCLE 1000000/TC4_LOOP_RATE                          //1000 at 1KHz
#define US_PER_TC4_TICK 1000000.0*2/48000000                            //41.6ns resolution

#endif
