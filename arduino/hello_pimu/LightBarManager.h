/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __LED_STRIP_MANAGER_H__
#define  __LED_STRIP_MANAGER_H__

#include "Common.h"

class LEDStripManager {    
  public: 
    LEDStripManager();
    void step();
    void setupLEDStripManager();
  private:
  int pix_id;

    
};

extern LEDStripManager strip_manager;

#endif
