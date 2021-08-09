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

class SyncManager {    
  public: 
    SyncManager();
    void trigger_motor_sync();
    void trigger_runstop();
    void clear_runstop();
    void step(Pimu_Status * stat);
  private:
    volatile uint16_t pulse_len_ms; 
    volatile bool runstop_active;
    volatile bool dirty_motor_sync;
};

extern SyncManager sync_manager;


#endif
