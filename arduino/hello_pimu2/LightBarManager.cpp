/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#include "LightBarManager.h"
#include "TimeManager.h"

#define NUM_PIXELS 4
#define MODE_LB_OFF 0
#define MODE_LB_BOOTING 1
#define MODE_LB_CONSTANT_COLOR 2
#define MODE_LB_CYCLING_COLOR 3


#define PX_GREEN 0,64,0
#define PX_YELLOW_GREEN 32,64,0
#define PX_YELLOW 64,64,0
#define PX_ORANGE 64,32,0 
#define PX_RED 64,0,0

#define PX_PINK 8,1,3
#define PX_OFF 0,0,0
#define PX_WHITE 32,32,32

#define V_BAT_MIN 23.5
#define V_BAT_MAX 25.6

//Light bar interpolates from RED to GREEN over range V_BAT_MIN-->V_BAT_MAX

uint8_t Red(uint32_t color){return (color >> 16) & 0xFF;}
uint8_t Green(uint32_t color){return (color >> 8) & 0xFF;}
uint8_t Blue(uint32_t color){return color & 0xFF;}

Adafruit_NeoPixel_ZeroDMA pixels(NUM_PIXELS, NEOPIXEL, NEO_GRB);

//Slew a pixel color from bg to fg and back over duration_ms
class SlewPixel
{
  public:
  float r,b,g;
  int rf,rb,bf,bb,gf,gb;
  float d_pct;
  float pct;
  bool configured;
  float pct_max;
  float pct_min;
  int dir;
  SlewPixel()
  {
    configured=false;
    dir=1;
  }
  void Configure(uint32_t color_bg, uint32_t color_fg, float duration_ms, float init_pct, float p_min, float p_max)
  {
    rf=Red(color_fg);
    rb=Red(color_bg);
    gf=Green(color_fg);
    gb=Green(color_bg);
    bf=Blue(color_fg);
    bb=Blue(color_bg);
    pct=init_pct;
    d_pct = (10/duration_ms)/(p_max-p_min);
    pct_min=p_min;
    pct_max=p_max;
    SetPct(init_pct);
    configured=true;
    dir=1;
  }
  void SetPct(float p) //pass in 0-1.0. O.0 is BG color 1.0 is FG color. Over 1.0 or <0.0 set to FG/BG (allows for delay at full color)
  {
    if(p>1.0) 
    {
      r=rf;
      g=gf;
      b=bf;
    }
    else if (p<0)
    {
      r=rb;
      g=gb;
      b=bb;
    }
    else
    {
      r=rb*(1-p)+rf*p;
      g=gb*(1-p)+gf*p;
      b=bb*(1-p)+bf*p;
    }
  }
  //Return true if at the end of a cycle so can update the color
  void Step()
  {
    
    pct=max(pct_min,min(pct_max,pct+d_pct*dir));

    //Every slew cycle reconfigure the FG/BG/rate
    if (pct==pct_min)
      configured=false;
    else
      configured=true;
      
    if ((pct==pct_min && dir==-1)||(pct==pct_max && dir==1))
      dir=dir*-1;
    SetPct(pct);
  }
};

uint32_t Color1, Color2;  // What colors are in use
SlewPixel p0, p1, p2, p3;

int test_soc;
LightBarManager::LightBarManager()
{
  lightBar_init=false;
}
void LightBarManager::Battery_Gauge(int soc,bool runstop_on, bool runstop_led_on,bool charger_on)
{
  if (runstop_led_on || !runstop_on)
  {
    if (!charger_on)
    {
      if (soc == 100){
          pixels.setPixelColor(0, pixels.Color(PX_GREEN));
          pixels.setPixelColor(1, pixels.Color(PX_GREEN));
          pixels.setPixelColor(2, pixels.Color(PX_GREEN));
          pixels.setPixelColor(3, pixels.Color(PX_GREEN));
          pixels.show();
      }
      if (soc == 75){
          pixels.setPixelColor(0, pixels.Color(PX_OFF));
          pixels.setPixelColor(1, pixels.Color(PX_GREEN));
          pixels.setPixelColor(2, pixels.Color(PX_GREEN));
          pixels.setPixelColor(3, pixels.Color(PX_GREEN));
          pixels.show();
      }
      if (soc == 50){
          pixels.setPixelColor(0, pixels.Color(PX_OFF));
          pixels.setPixelColor(1, pixels.Color(PX_OFF));
          pixels.setPixelColor(2, pixels.Color(PX_GREEN));
          pixels.setPixelColor(3, pixels.Color(PX_GREEN));
          pixels.show();
      }
      if (soc == 20)
      {
          pixels.setPixelColor(0, pixels.Color(PX_OFF));
          pixels.setPixelColor(1, pixels.Color(PX_OFF));
          pixels.setPixelColor(2, pixels.Color(PX_OFF));
          pixels.setPixelColor(3, pixels.Color(PX_YELLOW));
          pixels.show();
      }
      if (soc == 10)
      {
          pixels.setPixelColor(0, pixels.Color(PX_OFF));
          pixels.setPixelColor(1, pixels.Color(PX_OFF));
          pixels.setPixelColor(2, pixels.Color(PX_OFF));
          pixels.setPixelColor(3, pixels.Color(PX_RED));
          pixels.show();
      }
    }
    if (charger_on && !runstop_on)
    {
      ColoredScanUpdate(pixels.Color(PX_OFF),pixels.Color(PX_GREEN),1000);
    } 
  }
  else
  {
    pixels.clear();
    pixels.show();
  }
}
void LightBarManager::ColoredBatteryLevel(float v_bat, float v_bat_min, float v_bat_max,bool runstop_on, bool runstop_led_on,bool charger_on)
{
  if (runstop_led_on || !runstop_on)
    {
      float dv=(v_bat_max-v_bat_min)/4;
      uint32_t c1, c2;
      float interp;
      if (v_bat<v_bat_min)
      {
        c1=pixels.Color(PX_RED);
        c2=pixels.Color(PX_RED);
        interp=1.0;
      }
      else if (v_bat>=v_bat_min && v_bat<(v_bat_min+dv))
      {
        c1=pixels.Color(PX_RED);
        c2=pixels.Color(PX_ORANGE);
        interp = (v_bat-v_bat_min)/dv;
      }
      else if (v_bat>=(v_bat_min+dv) && v_bat<(v_bat_min+2*dv))
      {
        c1=pixels.Color(PX_ORANGE);
        c2=pixels.Color(PX_YELLOW);
        interp = (v_bat-(v_bat_min+dv))/dv;
      }
      else if (v_bat>=(v_bat_min+2*dv) && v_bat<(v_bat_min+3*dv))
      {
        c1=pixels.Color(PX_YELLOW);
        c2=pixels.Color(PX_YELLOW_GREEN);
        interp = (v_bat-(v_bat_min+2*dv))/dv;
      }
      else if (v_bat>=(v_bat_min+3*dv) && v_bat<v_bat_max)
      {
        c1=pixels.Color(PX_YELLOW_GREEN);
        c2=pixels.Color(PX_GREEN);
        interp = (v_bat-(v_bat_min+3*dv))/dv;
      }
      else
      {
        c1=pixels.Color(PX_GREEN);
        c2=pixels.Color(PX_GREEN);
        interp=1.0;
      }
      uint8_t r = ((float)(Red(c2)-Red(c1)))*interp+Red(c1);
      uint8_t g = ((float)(Green(c2)-Green(c1)))*interp+Green(c1);
      uint8_t b = ((float)(Blue(c2)-Blue(c1)))*interp+Blue(c1);
      if (charger_on && !runstop_on)
      {
        ColoredScanUpdate(pixels.Color(PX_OFF),pixels.Color(r,g,b),1000);
      } 
      else
      {
        ColorSet(pixels.Color(r,g,b));
        pixels.show();
      }
    }
    else
    {
      pixels.clear();
      pixels.show();
    }
}


void LightBarManager::ColorSet(uint32_t color)
{
  for (int i = 0; i < pixels.numPixels(); i++)
  {
      pixels.setPixelColor(i, color);
  }
  pixels.show();
}

void LightBarManager::Off()
{
  pixels.clear();
  pixels.show();
}

void LightBarManager::ColoredScanUpdate(uint32_t color_bg,uint32_t color_fg,float duration_ms)
{
    if(!p0.configured)
      p0.Configure(color_bg, color_fg, duration_ms, 0.0, 0, 1.0);
    p0.Step();

    if(!p1.configured)
      p1.Configure(color_bg, color_fg, duration_ms, 0.0, 0, 1.0);
     p1.Step();
     
    if(!p2.configured)
      p2.Configure(color_bg, color_fg, duration_ms, 0.0, 0, 1.0);
    p2.Step();

    if(!p3.configured)
      p3.Configure(color_bg, color_fg, duration_ms, 0.0, 0, 1.0);
    p3.Step();

    pixels.setPixelColor(0, pixels.Color(p0.r,p0.g,p0.b));
    pixels.setPixelColor(1, pixels.Color(p1.r,p1.g,p1.b));
    pixels.setPixelColor(2, pixels.Color(p2.r,p2.g,p2.b));
    pixels.setPixelColor(3, pixels.Color(p3.r,p3.g,p3.b));
    pixels.show();
}


void LightBarManager::setupLightBarManager()
{
   lightBar_init = pixels.begin(&sercom2, SERCOM2, SERCOM2_DMAC_ID_TX, NEOPIXEL, SPI_PAD_3_SCK_1, PIO_SERCOM);
}

void LightBarManager::disableDMAC()
{
  Off();
  delayMicroseconds(500);
  uint8_t chan = pixels.getDMA().getChannel();
  while(DMAC->Channel[chan].CHSTATUS.bit.BUSY);
  DMAC->Channel[chan].CHCTRLA.bit.ENABLE = 0; //Disable the channel
  while(DMAC->Channel[chan].CHCTRLA.bit.ENABLE);

  SERCOM2->SPI.CTRLA.bit.ENABLE = 0; //Disable the SERCOM SPI
  while(SERCOM2->SPI.SYNCBUSY.bit.ENABLE); //Wait for the SERCOM SPI to be disabled

}

void LightBarManager::enableDMAC()
{
   uint8_t chan = pixels.getDMA().getChannel();
  DMAC->Channel[chan].CHCTRLA.bit.ENABLE = 1; //Disable the channel
  while(!DMAC->Channel[chan].CHCTRLA.bit.ENABLE);

  SERCOM2->SPI.CTRLA.bit.ENABLE = 1; //Disable the SERCOM SPI
  while(!SERCOM2->SPI.SYNCBUSY.bit.ENABLE); //Wait for the SERCOM SPI to be disabled
}

float test_voltage=0.0;
bool running_test=false;

void LightBarManager::start_test()
{
  test_voltage=12.4;
  running_test=true;
}

void LightBarManager::step(bool boot_detected, bool runstop_on, bool charger_on, bool charging_required, bool runstop_led_on,float soc) 
{
  if (lightBar_init)
  {   
    
      if (running_test)
      {
        ColoredBatteryLevel(test_voltage, V_BAT_MIN, V_BAT_MAX, runstop_on, runstop_led_on, charger_on );
        test_voltage=max(23.0,test_voltage-0.005);
        if (test_voltage==23.0)
          running_test=false;
      }
      else
      {
        Battery_Gauge(soc, runstop_on, runstop_led_on, charger_on );
      }
  }
}
