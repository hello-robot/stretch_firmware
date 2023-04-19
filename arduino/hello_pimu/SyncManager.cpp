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

//This will generate a sync pulse on the sync or runstop line to the steppers
//It will also hold the runstop line high if there is an active runstop

//For steppers the sync will trigger on any low-->high transition

// Pulse profile: Max rate is 100hz for a pulse train
#define SYNC_PULSE_ON_SHARED 5 //ms
#define SYNC_PULSE_OFF_SHARED 5 //ms
#define SYNC_PULSE_ON_DEDICATED 5 //ms
#define SYNC_PULSE_OFF_DEDICATED 5 //ms

SyncManager::SyncManager(RunstopManager * r)
{
  pulse_len_ms=0;
  motor_stop_enabled=0;
  dirty_motor_sync=0;
  pulse_wait_ms=0;
  motor_sync_cnt=0;
  rm=r;
}

//Raise the sync trigger line as soon as get an RPC
void SyncManager::trigger_motor_sync() //Called aperiodically from RPC
{
  dirty_motor_sync=1;
  if(BOARD_VARIANT_DEDICATED_SYNC==0)
  {
    digitalWrite(RUNSTOP_M0, HIGH);
    digitalWrite(RUNSTOP_M1, HIGH);
    digitalWrite(RUNSTOP_M2, HIGH);
    digitalWrite(RUNSTOP_M3, HIGH);
  }
  if(BOARD_VARIANT_DEDICATED_SYNC>=1)
  {
    digitalWrite(SYNC_OUT, HIGH);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Raise runstop line if in runstop event 
//Raise sync line if in the on period of a sync pulse
//Otherwise lower line

void SyncManager::step_dedicated_sync()
{
    noInterrupts();
    
    //Handle runstop
    if (rm->state_runstop_event)
      digitalWrite(RUNSTOP_OUT, HIGH);//Disable motors
    else 
      digitalWrite(RUNSTOP_OUT, LOW);//Enable motors


    //Start a new sync pulse?
    if (!pulse_len_ms && !pulse_wait_ms && dirty_motor_sync ) //allow current pulse to finish before handling new event
    {
        pulse_len_ms=SYNC_PULSE_ON_DEDICATED+1; //+1 so step loop comes out correct
        dirty_motor_sync=0;
        motor_sync_cnt++;
    }   
    
    if (pulse_len_ms)
    {
      digitalWrite(SYNC_OUT, HIGH);
      pulse_len_ms=max(0,pulse_len_ms-1);
      if(pulse_len_ms==0)
        pulse_wait_ms=SYNC_PULSE_OFF_DEDICATED; //Ensure a pause between motor pulses,otherwise SW can spam line to look like a runstop
    }
    else 
      digitalWrite(SYNC_OUT, LOW);
    
    pulse_wait_ms=max(0,pulse_wait_ms-1);
    interrupts();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Raise runstop line if in runstop event or
//If in the on period of a sync pulse
//Otherwise lower line
void SyncManager::step_shared_sync()
{ 
    noInterrupts();

    
    //Start a new sync pulse?
    if (!pulse_len_ms && dirty_motor_sync && !rm->state_runstop_event && pulse_wait_ms==0) //allow current pulse to finish before handling new event
    {
        pulse_len_ms=SYNC_PULSE_ON_SHARED+1; //+1 so step loop comes out correct
        dirty_motor_sync=0;
        motor_sync_cnt++;
    }    
    
    if (rm->state_runstop_event || pulse_len_ms)
    {
      digitalWrite(RUNSTOP_M0, HIGH);
      digitalWrite(RUNSTOP_M1, HIGH);
      digitalWrite(RUNSTOP_M2, HIGH);
      digitalWrite(RUNSTOP_M3, HIGH);
      pulse_len_ms=max(0,pulse_len_ms-1);
      if(pulse_len_ms==0)
        pulse_wait_ms=SYNC_PULSE_OFF_SHARED; //Ensure a pause between motor pulses,otherwise SW can spam line to look like a runstop
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


void SyncManager::step() //Called at 1Khz from TC4 ISR
{
  if(BOARD_VARIANT_DEDICATED_SYNC)
    step_dedicated_sync();
  else
    step_shared_sync();
}
 
  
