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


class LightBarManager {    
  public: 
    LightBarManager();
    void step();
    void setupLightBarManager();
    void set_mode(uint8_t m);
  private:
  int pix_id;
  uint8_t brightness;
  uint8_t dir;
  Adafruit_NeoPixel_ZeroDMA * pixels;
  uint8_t mode;
  void step_mode_white_blink_slow();
  void step_mode_off();
  //void step_mode_cylon_bounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay);
  int idx_1;
  int idx_2;
};



#endif
