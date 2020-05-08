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
//extern void on_runstop_change();
extern float debug;
extern void setupMGInterrupts();
extern void enableMGInterrupts();
extern void disableMGInterrupts(); 
extern float FsCtrl;
#endif
