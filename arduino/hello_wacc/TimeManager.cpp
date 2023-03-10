/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#include "TimeManager.h"
#include "Common.h"

TimeManager time_manager;

///////////////////// TIMING and TIMESTAMP /////////////////////////////////////////
/*
 * TC4 is used to track time as micros() doesn't work in ISRs
 * TimeManage tracks TC4 overflows, computes timestamps as uS since last zero_clock()
 * zero_clock() is called by RPC from client
 * Duration measurement provided for testing of MotorSyncManager pulse generation
 * 
  */

unsigned long TimeManager::get_elapsed_time_ms()
{
  return (unsigned long)((float)(MS_PER_TC4_CYCLE*(float)ts_base));
} 

TimeManager::TimeManager()
{
  ts_base=0;
}
    
uint64_t TimeManager::current_time_us()
{
  uint64_t base;
  int cntr = TC4->COUNT16.COUNT.reg;
   base = ts_base*US_PER_TC4_CYCLE;
  return base+(int)(round(cntr*US_PER_TC4_TICK));
}

void TimeManager::clock_zero()
{
  ts_base=0;
  TC4->COUNT16.COUNT.reg=0;
}
