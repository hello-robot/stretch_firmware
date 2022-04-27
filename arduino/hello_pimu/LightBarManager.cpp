/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#include "LightBarManager.h"


#define NUM_PIXELS 4
#define MODE_LB_OFF 0
#define MODE_LB_WHITE_BLINK_SLOW 1
#define MODE_LB_CYLON_BOUNCE 2

LightBarManager::LightBarManager()
{
  pixels=0;
}
   

void LightBarManager::setupLightBarManager()
{
   pinMode(NEOPIXEL, OUTPUT);
   pixels = new Adafruit_NeoPixel_ZeroDMA(NUM_PIXELS, NEOPIXEL, NEO_GRB);
   pixels->begin(&sercom1, SERCOM1, SERCOM1_DMAC_ID_TX, NEOPIXEL, SPI_PAD_2_SCK_3, PIO_SERCOM_ALT);
   set_mode(MODE_LB_WHITE_BLINK_SLOW);
}


void LightBarManager::step_mode_off()
{
  pixels->clear();
}


void LightBarManager::step_mode_white_blink_slow()
{
  if(brightness<255 && dir==0) //ramp up
      brightness+=2;
    if(brightness>0 && dir==1) //ramp down
      brightness-=2;
    if(brightness==0)
      dir=0;
    if(brightness==254)
      dir=1;
    pixels->setBrightness(brightness+1);

}


void LightBarManager::set_mode(uint8_t m)
{
  if(!pixels)
    return;
  mode=m;
  if (mode==MODE_LB_OFF)
  {
    pixels->clear();
  }
  if (mode==MODE_LB_WHITE_BLINK_SLOW)
  {
    brightness=32;
    dir=0;
     pixels->setBrightness(brightness);
     pixels->setPixelColor(0, 255,255,255);
     pixels->setPixelColor(1, 255,255,255);
     pixels->setPixelColor(2, 255,255,255);
     pixels->setPixelColor(3, 255,255,255);
  }
  if (mode==MODE_LB_CYLON_BOUNCE)
  {
    idx_1=0;
    idx_2=1;
  }
  pixels->show();
}

void LightBarManager::step() //Called at 100hz from TC4 loop
{
  if (pixels) //Ignore if board variant w/o neopixel
  {
    switch(mode){
    case(MODE_LB_WHITE_BLINK_SLOW):
      step_mode_white_blink_slow();
      break;
    //case(MODE_LB_CYLON_BOUNCE):
    //  step_mode_cylon_bounce(255,100,100,1,10,50);
    case(MODE_LB_OFF):
      step_mode_off();
      break;
    };
    pixels->show();
  }
}


/*

void LightBarManager::step_mode_cylon_bounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){

  for(in = 0; i < NUM_PIXELS-EyeSize-2; i++) {
    pixels->clear();//setAll(0,0,0);
    pixels->setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      pixels->setPixelColor(idx_1+idx_2, red, green, blue);
    }
    pixels->setPixelColor(i+EyeSize+1, red/10, green/10, blue/10);
    pixels->show();
    delay(SpeedDelay);
  }

  delay(ReturnDelay);

  for(int i = NUM_PIXELS-EyeSize-2; i > 0; i--) {
    pixels->clear();//setAll(0,0,0);
    pixels->setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      pixels->setPixelColor(i+j, red, green, blue);
    }
    pixels->setPixelColor(i+EyeSize+1, red/10, green/10, blue/10);
    pixels->show();
    delay(SpeedDelay);
  }
 
  delay(ReturnDelay);
}*/
