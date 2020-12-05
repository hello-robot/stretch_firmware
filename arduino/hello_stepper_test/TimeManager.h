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
    void timestamp_encoder();
    void timestamp_status_sync();
    uint64_t get_encoder_timestamp();
    uint64_t get_status_sync_timestamp();
    void setupTimeManager();
    volatile int ts_base; //Incremented once every TC4 loop

  private:
    
    volatile int64_t encoder_ts_base;
    volatile int32_t encoder_ts_cntr;
    volatile int64_t status_sync_ts_base;
    volatile int32_t status_sync_ts_cntr;
    float dt_ms;

};

extern TimeManager time_manager;
#endif
