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


///////////////////////// SYNCMANAGER ///////////////////////////


SyncManager sync_manager;

SyncManager::SyncManager()
{
  runstop_active=1;
  pulse_count=0;
  rs_last=0;
  sync_last=0;
  sync_mode_enabled = false;
  motor_sync_triggered=false;
  last_pulse_duration=0;
  runstop_trigger_cnt=0;
}


// For BOARD_VARIANT==0:
// Always enable runstop if runstop line is high for over 80 ms
// Always trigger sync if a low to high transition on runstop line

// For BOARD_VARIANT==1 & 2:
// Always enable runstop if runstop line is high for over 80 ms
// Always trigger sync if a low to high transition on sync line


#define RUNSTOP_TRIGGER_MS 80


void  SyncManager::step() //Called at 1Khz from TC4 loop
{

   if (BOARD_VARIANT==0 || BOARD_VARIANT>=1)
   {
    //Poll line at 1Khz
      uint8_t rs=digitalRead(BOARD_VARIANT_PIN_RUNSTOP);
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
        }
        runstop_active=0;
        pulse_count=0;
      }
    rs_last=rs;
   }
}


void handleIRQ()
{
  sync_manager.irq_cnt++;
  if (BOARD_VARIANT==0 || BOARD_VARIANT>=1)
  {
      if(sync_manager.sync_mode_enabled)
        sync_manager.motor_sync_triggered=true;
  }
}


void SyncManager::setupSyncManager() {  
  rs_last=digitalRead(BOARD_VARIANT_PIN_RUNSTOP);
  if (BOARD_VARIANT>=1)
    sync_last=digitalRead(PIN_SYNC);
  if (BOARD_VARIANT==0)
  {
    attachInterrupt(digitalPinToInterrupt(BOARD_VARIANT_PIN_RUNSTOP), handleIRQ, RISING);
  }
  if (BOARD_VARIANT>=1)
  {
    attachInterrupt(digitalPinToInterrupt(PIN_SYNC), handleIRQ, RISING);
  }
  irq_cnt=0;
}
