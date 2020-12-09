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

#define SYNC_PULSE_STATUS_ONLY 2 //ms
#define SYNC_PULSE_STATUS_MOTOR 4 //ms

SyncManager sync_manager;


SyncManager::SyncManager()
{
  pulse_len_ms=0;
  dirty_status_sync=0;
  runstop_active=0;
  dirty_motor_sync=0;
  pulse_polarity=0; //O is high pulse, 1 is low pulse
}

void SyncManager::trigger_status_sync() //Called aperiodically from RPC
{
  dirty_status_sync=1;
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

void SyncManager::step(Pimu_Status * stat_sync, Pimu_Status * stat_out, Pimu_Config * cfg) //Called at 1Khz from TC4 ISR
{
  noInterrupts();
  if (!pulse_len_ms && (dirty_motor_sync || dirty_status_sync)) //allow current pulse to finish before handling new event
  {
     pulse_polarity=runstop_active; //Latch in the runstop state at the time the event was triggered

    if (dirty_motor_sync)
      pulse_len_ms=SYNC_PULSE_STATUS_MOTOR+1; //+1 so step loop comes out correct
    else if (dirty_status_sync)
      pulse_len_ms=SYNC_PULSE_STATUS_ONLY+1; //+1 so step loop comes out correct

    dirty_status_sync=0;
    dirty_motor_sync=0;
    memcpy((uint8_t *)stat_sync,(uint8_t *)stat_out,sizeof(Pimu_Status)); //Cache most recent status
    stat_sync->timestamp_line_sync=time_manager.current_time_us(); //Mark time of new sync event
    time_manager.start_duration_measure();
      
  }    
  interrupts();

  if(pulse_len_ms && cfg->sync_mode_enabled)
  {
      if(pulse_polarity==0)
      {
        digitalWrite(RUNSTOP_M0, HIGH);
        digitalWrite(RUNSTOP_M1, HIGH);
        digitalWrite(RUNSTOP_M2, HIGH);
        digitalWrite(RUNSTOP_M3, HIGH);
      }
      else
      {
        digitalWrite(RUNSTOP_M0, LOW);
        digitalWrite(RUNSTOP_M1, LOW);
        digitalWrite(RUNSTOP_M2, LOW);
        digitalWrite(RUNSTOP_M3, LOW);
      }
    pulse_len_ms=pulse_len_ms-1;
    if (pulse_len_ms==0) //Done with pulse
    {
        if(pulse_polarity==1)
      {
        digitalWrite(RUNSTOP_M0, HIGH);
        digitalWrite(RUNSTOP_M1, HIGH);
        digitalWrite(RUNSTOP_M2, HIGH);
        digitalWrite(RUNSTOP_M3, HIGH);
      }
      else
      {
        digitalWrite(RUNSTOP_M0, LOW);
        digitalWrite(RUNSTOP_M1, LOW);
        digitalWrite(RUNSTOP_M2, LOW);
        digitalWrite(RUNSTOP_M3, LOW);
      }
         stat_sync->debug=time_manager.end_duration_measure();
    }
    return;
  }
  
  if(runstop_active)
  {
    digitalWrite(RUNSTOP_M0, HIGH);
    digitalWrite(RUNSTOP_M1, HIGH);
    digitalWrite(RUNSTOP_M2, HIGH);
    digitalWrite(RUNSTOP_M3, HIGH);
  }
  else
  {
    digitalWrite(RUNSTOP_M0, LOW);
    digitalWrite(RUNSTOP_M1, LOW);
    digitalWrite(RUNSTOP_M2, LOW);
    digitalWrite(RUNSTOP_M3, LOW);
  }
}
