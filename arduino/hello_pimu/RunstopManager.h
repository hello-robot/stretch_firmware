/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __RUNSTOP_MANAGER_H__
#define  __RUNSTOP_MANAGER_H__

#include "Common.h"

class RunstopManager {    
  public: 
    RunstopManager();
    void activate_runstop();
    void deactivate_runstop();
    void step(Pimu_Config * cfg);
    void toggle_led(int rate_ms);
    bool state_runstop_event;
    bool runstop_led_on;
  private:
    bool trigger_runstop_event;
    volatile uint8_t runstop_mode;
    unsigned long runstop_t_toggle_last;
    int depressed_last;
    unsigned long t_low;
    
};


#endif
