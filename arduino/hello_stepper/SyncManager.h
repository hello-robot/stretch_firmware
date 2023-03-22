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

#ifndef __HELLO_SYNC_MANAGER_H__
#define  __HELLO_SYNC_MANAGER_H__

#include "Common.h"

/* For RE1   (BOARD_VARIANT==0) the sync line and runstop line are shared
 * For RE2 (BOARD_VARIANT==1) the lines are seperated
 * We handle each case seperately 
 */
class SyncManager{
   public: 
    SyncManager();
    volatile bool runstop_active;
    volatile bool motor_sync_triggered;
    void setupSyncManager();
    void step();
    bool sync_mode_enabled;
    int runstop_trigger_cnt;
    void on_runstop_change();
    void on_sync_change();
    int irq_cnt;
  private:
    uint8_t rs_last;
    int last_pulse_duration;
    int pulse_count;
    bool sync_last;

};

extern SyncManager sync_manager;

#endif
