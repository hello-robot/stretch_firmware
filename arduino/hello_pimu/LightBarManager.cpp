/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#include "LEDStripManager.h"
//#include <Adafruit_NeoPixel_ZeroDMA.h>
#include <Adafruit_NeoPixel.h>


#define NUM_PIXELS 4


  //if Mitski
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL, NEO_GRB);
LEDStripManager strip_manager;

LEDStripManager::LEDStripManager()
{
  
}
   

void LEDStripManager::setupLEDStripManager()
{
  strip.begin();
  pix_id=0;
}
int shift_idx=0;
void LEDStripManager::step() //Called at 100hz from TC4 loop
{

  // 'Color wipe' across all pixels
    uint32_t c = 0xFF0000>>shift_idx;
    strip.setPixelColor(pix_id, c);
    strip.show();
    delay(50);
    pix_id=pix_id+1;
    shift_idx=shift_idx+8;
    if(pix_id==NUM_PIXELS)
    {
      pix_id=0;
      shift_idx=8;
    }
    
 }
