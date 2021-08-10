/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#include "SyncManager.h"
#include "Common.h"
#include "TimeManager.h"


#define SYNC_PULSE_MOTOR 40 //ms




SyncManager::SyncManager(RunstopManager * r)
{
  pulse_len_ms=0;
  motor_stop_enabled=0;
  dirty_motor_sync=0;
  sync_cntr=0;
  rm=r;
}

void SyncManager::trigger_motor_sync() //Called aperiodically from RPC
{
  dirty_motor_sync=1;
}


void SyncManager::step(Pimu_Status * stat) //Called at 1Khz from TC4 ISR
{
  //Disable motors or generate sync pulse to trigger motors  
  noInterrupts();
  if (!pulse_len_ms && dirty_motor_sync && !rm->state_runstop_event) //allow current pulse to finish before handling new event
  {
      pulse_len_ms=SYNC_PULSE_MOTOR+1; //+1 so step loop comes out correct
      dirty_motor_sync=0;
      //time_manager.start_duration_measure();
      stat->debug=++sync_cntr;
  }    
  
  if (rm->state_runstop_event || pulse_len_ms)
  {
    digitalWrite(RUNSTOP_M0, HIGH);
    digitalWrite(RUNSTOP_M1, HIGH);
    digitalWrite(RUNSTOP_M2, HIGH);
    digitalWrite(RUNSTOP_M3, HIGH);

    pulse_len_ms=max(0,pulse_len_ms-1);
  }
  else 
  {//Enable motors
      digitalWrite(RUNSTOP_M0, LOW);
      digitalWrite(RUNSTOP_M1, LOW);
      digitalWrite(RUNSTOP_M2, LOW);
      digitalWrite(RUNSTOP_M3, LOW);
  }
  interrupts();
}
