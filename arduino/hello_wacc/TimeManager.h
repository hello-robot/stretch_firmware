/*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __TIME_MANAGER_H__
#define  __TIME_MANAGER_H__

#include "Common.h"

#define FS_CTRL 1000
#define US_PER_TC4_CYCLE 1000000/FS_CTRL //1000 at 1000Hz
#define MS_PER_TC4_CYCLE US_PER_TC4_CYCLE/1000
#define TC4_TICKS_PER_CYCLE (int)( round(48000000 / 2 / FS_CTRL)) //24,000 at 1000hz, 2:1 prescalar TC4 is 32bit timer
#define US_PER_TC4_TICK 1000000.0*2/48000000 


class TimeManager {
   public: 
    TimeManager();
    uint64_t current_time_us();
    void clock_zero();
    unsigned long get_elapsed_time_ms();
    volatile int ts_base; //Incremented at rate FS_CTRL (Hz)
};

extern TimeManager time_manager;

#endif
