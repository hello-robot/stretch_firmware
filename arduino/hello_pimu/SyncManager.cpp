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
#include "Pimu.h"

#define SYNC_PULSE_MOTOR 40 //ms
#define WAIT_PULSE_MOTOR 40 //ms
//For variant_1 steppers the sync will trigger on any low-->high transition
//so pulse width isn't defining. 
//Making it short to allow for ~100hz max sync rate (higher than needed)
#define SYNC_PULSE_MOTOR_DEDICATED_SYNC 10 //ms

SyncManager::SyncManager(RunstopManager * r)
{
  pulse_len_ms=0;
  motor_stop_enabled=0;
  dirty_motor_sync=0;
  pulse_wait_ms=0;
  rm=r;
}

void SyncManager::trigger_motor_sync() //Called aperiodically from RPC
{
  dirty_motor_sync=1;
}


void SyncManager::step() //Called at 1Khz from TC4 ISR
{
  
  if(BOARD_VARIANT_DEDICATED_SYNC==0)
  {
    //Variant 0: 
    //Runstop line: held high >80ms = activate runstop on steppers
    //Runstop line: 40ms pulse: trigger sync on steppers
    //Sync pulses can not happen faster than every 80ms (in order to avoid false runstop)
  
    //Disable motors or generate sync pulse to trigger motors  
    noInterrupts();
    if (!pulse_len_ms && dirty_motor_sync && !rm->state_runstop_event && pulse_wait_ms==0) //allow current pulse to finish before handling new event
    {
        pulse_len_ms=SYNC_PULSE_MOTOR+1; //+1 so step loop comes out correct
        dirty_motor_sync=0;
        time_manager.start_duration_measure();
    }    
    
    if (rm->state_runstop_event || pulse_len_ms)
    {
      digitalWrite(RUNSTOP_M0, HIGH);
      digitalWrite(RUNSTOP_M1, HIGH);
      digitalWrite(RUNSTOP_M2, HIGH);
      digitalWrite(RUNSTOP_M3, HIGH);
      if(pulse_len_ms==1)
      {
        duration_last_pulse=time_manager.end_duration_measure();
        pulse_wait_ms=WAIT_PULSE_MOTOR; //Ensure a pause between motor pulses,otherwise SW can spam line to look like a runstop
      }
      pulse_len_ms=max(0,pulse_len_ms-1);
     
    }
    else 
    {//Enable motors
        digitalWrite(RUNSTOP_M0, LOW);
        digitalWrite(RUNSTOP_M1, LOW);
        digitalWrite(RUNSTOP_M2, LOW);
        digitalWrite(RUNSTOP_M3, LOW);
    }
    pulse_wait_ms=max(0,pulse_wait_ms-1);
    interrupts();
  }
  
  if(BOARD_VARIANT_DEDICATED_SYNC==1)
  {
    //Variant 1: 
    //Runstop line: held high = activate runstop on steppers
    //Sync line: 40ms pulse: trigger sync on steppers

 
    noInterrupts();
    //Handle runstop
    if (rm->state_runstop_event)
    {
      digitalWrite(RUNSTOP_OUT, HIGH);//Disable motors
    }
    else 
    {
        digitalWrite(RUNSTOP_M0, LOW);//Enable motors
    }

    //Handle sync
    if (!pulse_len_ms && dirty_motor_sync) //
    {
        pulse_len_ms=SYNC_PULSE_MOTOR_DEDICATED_SYNC+1; //+1 so step loop comes out correct
        dirty_motor_sync=0;
        time_manager.start_duration_measure();
    }   
    if (pulse_len_ms)
    {
      digitalWrite(SYNC_OUT, HIGH);
    }
    else 
    {
        digitalWrite(SYNC_OUT, LOW);
    }
    pulse_len_ms=max(0,pulse_len_ms-1);
    if(pulse_len_ms==0)
    {
      duration_last_pulse=time_manager.end_duration_measure();
    }
    interrupts();
  }
}
