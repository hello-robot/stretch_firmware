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

#include "SyncManager.h"
#include "HelloController.h"
#include "Controller.h"
#include "TimeManager.h"
///////////////////////// RUNSTOP & MOTOR SYNC ///////////////////////////


SyncManager sync_manager;

SyncManager::SyncManager()
{
  runstop_active=1;
  pulse_count=0;
  rs_last=0;
  sync_mode_enabled = false;
  motor_sync_triggered=false;
  last_pulse_duration=0;
  runstop_trigger_cnt=0;
}
    
//Pimu will generate a 40ms pulse on the runstop line to signal a motor sync
//If the line remains high longer than 80ms, then signal a runstop trigger
//Accept a pulse range of SYNC_PULSE_MIN_MS to SYNC_PULSE_MAX_MS

#define SYNC_PULSE_MIN_MS 30 
#define SYNC_PULSE_MAX_MS 50 
#define RUNSTOP_TRIGGER_MS 80


void  SyncManager::step() //Called at 1Khz from TC4 loop
{
   
  //Poll line at 1Khz
    uint8_t rs=digitalRead(RUNSTOP);
    if (rs)
    {
      pulse_count=min(pulse_count+1,RUNSTOP_TRIGGER_MS); //count how long has been high
      if(pulse_count==RUNSTOP_TRIGGER_MS && !runstop_active)
      {
        runstop_active=1;
        runstop_trigger_cnt++;
        last_pulse_duration=RUNSTOP_TRIGGER_MS;
      }    
    }
    else
    {
      if(rs_last) //falling ege
      {
        last_pulse_duration=pulse_count;
        if(sync_mode_enabled && pulse_count>SYNC_PULSE_MIN_MS && pulse_count<SYNC_PULSE_MAX_MS)
          motor_sync_triggered=true;
        else
          motor_sync_triggered=false;
      }
      runstop_active=0;
      pulse_count=0;
    }
    rs_last=rs;
}
/////////////////////////////////////////////////////////////////


void SyncManager::setupSyncManager() {  
  
}
