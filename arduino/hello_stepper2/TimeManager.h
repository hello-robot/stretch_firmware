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

#ifndef __TIME_MANAGER_H__
#define  __TIME_MANAGER_H__

#include "Common.h"


class TimeManager {
   public: 
    TimeManager();
    uint32_t get_elapsed_time_ms();
    uint64_t current_time_us();
    void setupTimeManager();
    volatile uint64_t ts_base; //Incremented once every TC4 loop (1Khz)
    
  private:
    float dt_ms;

};

extern TimeManager time_manager;
#endif
