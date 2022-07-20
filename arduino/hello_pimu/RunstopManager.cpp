/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#include "RunstopManager.h"
#include "TimeManager.h"
#include "BeepManager.h"

#include "Common.h"



#define MODE_RUNSTOP_NOT_ACTIVE 0
#define MODE_RUNSTOP_ACTIVE 1
#define MODE_RESET_ACTIVE 2
//Runstop is pulled high
//Tapping button pulls line low (activates runstop)
//Holding button down for 2s releases runstop


RunstopManager::RunstopManager()
{
  runstop_mode=MODE_RUNSTOP_NOT_ACTIVE; //Runstop manager mode
  runstop_led_on=false;                 //LED state
  runstop_t_toggle_last=0;              //Time led was last toggled
  depressed_last=0;                     //Time button was last depressed
  trigger_runstop_event=0;              //External event triggered runstop (RPC, low volage)
  state_runstop_event=false;            //Runstop active + YAML enabled
}
   
void RunstopManager::activate_runstop() //Called from TC4 ISR
{
  trigger_runstop_event=1;
}

void RunstopManager::deactivate_runstop() //Called from TC4 ISR
{
  runstop_mode=MODE_RUNSTOP_NOT_ACTIVE;
  state_runstop_event=0;
  trigger_runstop_event=0;
}


void RunstopManager::step(Pimu_Config * cfg) //Called at 100hz from TC4 loop
{
  int button_depressed=(digitalRead(RUNSTOP_SW)==0);

  if (!depressed_last && button_depressed  || trigger_runstop_event) //button pushed
  {
    
    if (runstop_mode==MODE_RUNSTOP_NOT_ACTIVE)
    {
      runstop_mode=MODE_RUNSTOP_ACTIVE;
      beep_manager.do_beep(BEEP_ID_SINGLE_SHORT);
    }
    else if (runstop_mode==MODE_RUNSTOP_ACTIVE && !trigger_runstop_event) //already triggered, not an alert, so must be a start of a reset
    {
      t_low=time_manager.get_elapsed_time_ms();
      runstop_mode=MODE_RESET_ACTIVE;
    }
    trigger_runstop_event=0;
  }
  else 
  {
    if (runstop_mode==MODE_RESET_ACTIVE && button_depressed && time_manager.get_elapsed_time_ms()-t_low>2000)
    {
      runstop_mode=MODE_RUNSTOP_NOT_ACTIVE;
      beep_manager.do_beep(BEEP_ID_SINGLE_SHORT);
      runstop_t_toggle_last=0; 
    }
    else
    if(!button_depressed && runstop_mode==MODE_RESET_ACTIVE) //Not held down long enough
    {
      runstop_mode=MODE_RUNSTOP_ACTIVE;
    }
  }
  depressed_last=button_depressed;
  
  if(runstop_mode==MODE_RUNSTOP_NOT_ACTIVE)
    state_runstop_event=0;
  else
    state_runstop_event=cfg->stop_at_runstop;

}

void RunstopManager::toggle_led(int rate_ms)
{
  unsigned long t = time_manager.get_elapsed_time_ms();
  if (t-runstop_t_toggle_last>rate_ms)
  {
    runstop_t_toggle_last=t;
    if (!runstop_led_on)
    {
        digitalWrite(RUNSTOP_LED, HIGH);
    }
    else
    {
      digitalWrite(RUNSTOP_LED, LOW);
    }
   runstop_led_on=!runstop_led_on;
  }
}
