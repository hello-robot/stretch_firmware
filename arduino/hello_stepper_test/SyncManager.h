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


class SyncManager{
   public: 
    SyncManager();
    
    volatile bool runstop_active;
    volatile bool motor_sync_triggered;

    void setupSyncManager();
    void start_pulse_measure();
    void end_pulse_measure();
    void step();
    
    int last_pulse_duration;
    bool sync_mode_enabled;
    
    bool in_pulse;

    private:
    
    void enableTC3Interrupts();
    void disableTC3Interrupts();
    
    volatile int sync_duration_base_start;
    volatile int sync_duration_cntr_start;
    volatile int sync_duration_base_end;
    volatile int sync_duration_cntr_end;
    
    
    int pulse_start_count;
    
};

extern SyncManager sync_manager;
#endif
