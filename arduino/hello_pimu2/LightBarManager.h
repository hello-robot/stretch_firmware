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
#include "ChargerManager.h"
#include <Adafruit_NeoPixel_ZeroDMA.h>

enum  lightbar_mode {OFF,BOOTING, CHARGING_REQUIRED, CHARGING_RUNSTOP_ON, CHARGING_RUNSTOP_OFF, NORMAL_RUNSTOP_OFF, NORMAL_RUNSTOP_ON };
class LightBarPatterns;
class LightBarManager {    
  public: 
    LightBarManager();
    void step(bool boot_detected, bool runstop_on, bool charger_on, bool charging_required, bool runstop_led_on, uint8_t soc);
    void setupLightBarManager();
    uint8_t get_mode(){return (uint8_t) mode;}
    void start_test();
    void Off();
    void disableDMAC();
    void enableDMAC();
    void sleep_chrg();
    void low_battery_fault();
    
  private:
    void ColorSet(uint32_t color);
    void ColoredScanUpdate(uint32_t color_bg,uint32_t color_fg,float duration_ms);
    void ColoredBatteryLevel(float v_bat, float v_bat_min, float v_bat_max,bool runstop_on, bool runstop_led_on,bool charger_on);
    void Battery_Gauge(uint8_t soc,bool runstop_on, bool runstop_led_on,bool charger_on);
    bool lightBar_init;
    lightbar_mode mode; 
};

class helloDMA : public Adafruit_ZeroDMA
{
  public:
    uint8_t _dmac_channel;
    uint8_t getChannel() { return _dmac_channel; }
};



#endif
