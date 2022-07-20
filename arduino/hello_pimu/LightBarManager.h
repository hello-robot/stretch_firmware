/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __LIGHT_BAR_MANAGER_H__
#define  __LIGHT_BAR_MANAGER_H__

#include "Common.h"
#include <Adafruit_NeoPixel_ZeroDMA.h>

enum  lightbar_mode {OFF,BOOTING, CHARGING_REQUIRED, CHARGING_RUNSTOP_ON, CHARGING_RUNSTOP_OFF, NORMAL_RUNSTOP_OFF, NORMAL_RUNSTOP_ON };
class LightBarPatterns;
class LightBarManager {    
  public: 
    LightBarManager();
    void step(bool boot_detected, bool runstop_on, bool charger_on, bool charging_required, bool runstop_led_on, float v_bat);
    void setupLightBarManager();
    uint8_t get_mode(){return (uint8_t) mode;}
  private:
  LightBarPatterns * pixels;
  lightbar_mode mode; 
};



#endif
