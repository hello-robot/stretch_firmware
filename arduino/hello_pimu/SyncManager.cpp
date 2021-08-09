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

SyncManager sync_manager;


SyncManager::SyncManager()
{
  pulse_len_ms=0;
  runstop_active=0;
  dirty_motor_sync=0;
}

void SyncManager::trigger_motor_sync() //Called aperiodically from RPC
{
  dirty_motor_sync=1;
}

void SyncManager::trigger_runstop() //Called aperiodically from RPC or 
{
  runstop_active=1;
}

void SyncManager::clear_runstop()
{
  runstop_active=0;
}

void SyncManager::step(Pimu_Status * stat) //Called at 1Khz from TC4 ISR
{
  noInterrupts();
  if (!pulse_len_ms && dirty_motor_sync) //allow current pulse to finish before handling new event
  {
      pulse_len_ms=SYNC_PULSE_MOTOR+1; //+1 so step loop comes out correct
      dirty_motor_sync=0;
      time_manager.start_duration_measure();
  }    
  interrupts();

  if (runstop_active || pulse_len_ms)
  {//Disable motors or generate sync pulse to trigger motors
    digitalWrite(RUNSTOP_M0, HIGH);
    digitalWrite(RUNSTOP_M1, HIGH);
    digitalWrite(RUNSTOP_M2, HIGH);
    digitalWrite(RUNSTOP_M3, HIGH);
    pulse_len_ms=pulse_len_ms-1;
    if(pulse_len_ms==0)
    {
      stat->debug=time_manager.end_duration_measure();
      digitalWrite(RUNSTOP_M0, LOW);
      digitalWrite(RUNSTOP_M1, LOW);
      digitalWrite(RUNSTOP_M2, LOW);
      digitalWrite(RUNSTOP_M3, LOW);
    }
  }
  else 
  {//Enable motors
      digitalWrite(RUNSTOP_M0, LOW);
      digitalWrite(RUNSTOP_M1, LOW);
      digitalWrite(RUNSTOP_M2, LOW);
      digitalWrite(RUNSTOP_M3, LOW);
  }
}
