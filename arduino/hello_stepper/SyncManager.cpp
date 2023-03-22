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
  attachInterrupt(digitalPinToInterrupt(BOARD_VARIANT_PIN_RUNSTOP), handleIRQ, RISING);
  irq_cnt=0;
}


// For BOARD_VARIANT==0:
//Pimu will generate a 40ms pulse on the runstop line to signal a motor sync
//If the line remains high longer than 80ms, then signal a runstop trigger
//Accept a pulse range of SYNC_PULSE_MIN_MS to SYNC_PULSE_MAX_MS

// For BOARD_VARIANT==1:
// Always enable runstop if line is low
// Always trigger sync if a low to high transition

#define SYNC_PULSE_MIN_MS 30 
#define SYNC_PULSE_MAX_MS 50 
#define RUNSTOP_TRIGGER_MS 80


void  SyncManager::step() //Called at 1Khz from TC4 loop
{

   if (BOARD_VARIANT==0)
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


   if (BOARD_VARIANT==1 || BOARD_VARIANT==2)
   {
    //Poll line at 1Khz
      uint8_t rs=digitalRead(BOARD_VARIANT_PIN_RUNSTOP);
      if (rs)
      {
        pulse_count=min(pulse_count+1,RUNSTOP_TRIGGER_MS); //count how long has been high
        if(pulse_count==RUNSTOP_TRIGGER_MS && !runstop_active)
          runstop_active=1;
      }
      else
      {
        runstop_active=0;
        pulse_count=0;
      }
    rs_last=rs;
    
    uint8_t sync=digitalRead(PIN_SYNC);
    if (sync && !sync_last && sync_mode_enabled )
      motor_sync_triggered=true;
    else
      motor_sync_triggered=false;
    sync_last=sync;
    
   }
}

void handleIRQ()
{
  sync_manager.irq_cnt++;
  if (BOARD_VARIANT==0)
  {
      if(sync_manager.sync_mode_enabled)
        sync_manager.motor_sync_triggered=true;
}
}

void SyncManager::setupSyncManager() {
  rs_last=digitalRead(BOARD_VARIANT_PIN_RUNSTOP);
  if (BOARD_VARIANT==1 || BOARD_VARIANT==2)
    sync_last=digitalRead(PIN_SYNC);

  attachInterrupt(digitalPinToInterrupt(BOARD_VARIANT_PIN_RUNSTOP), handleIRQ, RISING);
  irq_cnt=0;
}

void SyncManager::setupSyncManager() {  
  rs_last=digitalRead(BOARD_VARIANT_PIN_RUNSTOP);
  if (BOARD_VARIANT==1 || BOARD_VARIANT==2)
    sync_last=digitalRead(PIN_SYNC);
}
