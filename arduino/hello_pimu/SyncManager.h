/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __SYNC_MANAGER_H__
#define  __SYNC_MANAGER_H__

#include "Common.h"
#include "RunstopManager.h"

class SyncManager {    
  public: 
    SyncManager(RunstopManager * r);
    void trigger_motor_sync();
    void step();
    volatile uint16_t pulse_len_ms;
    int duration_last_pulse;
    uint16_t motor_sync_cnt;
  private:
    void step_shared_sync();
    void step_dedicated_sync();
    volatile bool motor_stop_enabled;
    volatile bool dirty_motor_sync;
    int pulse_wait_ms;
    RunstopManager * rm;
};



#endif
