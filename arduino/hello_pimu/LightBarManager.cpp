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

#define V_BAT_MIN 10.0
#define V_BAT_MAX 12.0

//Light bar interpolates from RED to GREEN over range V_BAT_MIN-->V_BAT_MAX

uint8_t Red(uint32_t color){return (color >> 16) & 0xFF;}
uint8_t Green(uint32_t color){return (color >> 8) & 0xFF;}
uint8_t Blue(uint32_t color){return color & 0xFF;}

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
  int dir;
  SlewPixel()
  {
    configured=false;
    dir=1;
  }
  void Configure(uint32_t color_bg, uint32_t color_fg, float duration_ms, float init_pct, float pm)
  {
    rf=Red(color_fg);
    rb=Red(color_bg);
    gf=Green(color_fg);
    gb=Green(color_bg);
    bf=Blue(color_fg);
    bb=Blue(color_bg);
    pct=init_pct;
    d_pct = 10/duration_ms;
    pct_max=pm;
    SetPct(init_pct);
    configured=true;
    dir=1;
  }
  void SetPct(float p) //pass in 0-1.0. O.0 is BG color 1.0 is FG color. Over 1.0 turn off (to allow for a delay before starting new cycle)
  {
    if(p>1.0) 
    {
      r=0;
      g=0;
      b=0;
    }
    else
    {
      r=rb*(1-p)+rf*p;
      g=gb*(1-p)+gf*p;
      b=bb*(1-p)+bf*p;
    }
  }
  //Return true if at the end of a cycle so can update the color
  bool Step()
  {
    
    pct=max(0,min(pct_max,pct+d_pct*dir));
    if ((pct==0 && dir==-1)||(pct==pct_max && dir==1))
      dir=dir*-1;
    SetPct(pct);
    if(pct==pct_max || !configured)
    {
        return true;
        configured=true;
    }
    return false;
  }
};


class LightBarPatterns : public Adafruit_NeoPixel_ZeroDMA
{
    public:
    LightBarPatterns(uint16_t pixels, uint8_t pin, uint8_t type)
    :Adafruit_NeoPixel_ZeroDMA(pixels, pin, type)
    {
     
    }
    
    uint32_t Color1, Color2;  // What colors are in use
    SlewPixel p0, p1, p2, p3;
  
    void Off()
    {
      clear();
      show();
    }
    
    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color)
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, color);
        }
        show();
    }

//////////////////////////////////////////////////////////////////////  
  void ColoredScan(uint32_t color_bg,uint32_t color_fg,float duration_ms)
  {
    p0.Configure(color_bg, color_fg, duration_ms, 0.25, 1.5);
    p1.Configure(color_bg, color_fg, duration_ms, .5,1.5);
    p2.Configure(color_bg, color_fg, duration_ms, .75,1.5);
    p3.Configure(color_bg, color_fg, duration_ms, 1.0,1.5);
  }
  bool ColoredScanUpdate()
  {
    
    bool finished_cycle=p0.Step();
    p1.Step();
    p2.Step();
    p3.Step();
    setPixelColor(0, Color(p0.r,p0.g,p0.b));
    setPixelColor(1, Color(p1.r,p1.g,p1.b));
    setPixelColor(2, Color(p2.r,p2.g,p2.b));
    setPixelColor(3, Color(p3.r,p3.g,p3.b));
    show();
    return finished_cycle;
  }

////////////////////////////////////////////////////////////////////// 
  
  void ColoredBatteryLevel(float v_bat, float v_bat_min, float v_bat_max,bool runstop_on, bool runstop_led_on,bool charger_on)
  {
    if (runstop_led_on || !runstop_on)
    {
      float dv=(v_bat_max-v_bat_min)/4;
      uint32_t c1, c2;
      float interp;
      if (v_bat<v_bat_min)
      {
        c1=Color(PX_RED);
        c2=Color(PX_RED);
        interp=1.0;
      }
      else if (v_bat>=v_bat_min && v_bat<(v_bat_min+dv))
      {
        c1=Color(PX_RED);
        c2=Color(PX_ORANGE);
        interp = (v_bat-v_bat_min)/dv;
      }
      else if (v_bat>=(v_bat_min+dv) && v_bat<(v_bat_min+2*dv))
      {
        c1=Color(PX_ORANGE);
        c2=Color(PX_YELLOW);
        interp = (v_bat-(v_bat_min+dv))/dv;
      }
      else if (v_bat>=(v_bat_min+2*dv) && v_bat<(v_bat_min+3*dv))
      {
        c1=Color(PX_YELLOW);
        c2=Color(PX_YELLOW_GREEN);
        interp = (v_bat-(v_bat_min+2*dv))/dv;
      }
      else if (v_bat>=(v_bat_min+3*dv) && v_bat<v_bat_max)
      {
        c1=Color(PX_YELLOW_GREEN);
        c2=Color(PX_GREEN);
        interp = (v_bat-(v_bat_min+3*dv))/dv;
      }
      else
      {
        c1=Color(PX_GREEN);
        c2=Color(PX_GREEN);
        interp=1.0;
      }
      uint8_t r = ((float)(Red(c2)-Red(c1)))*interp+Red(c1);
      uint8_t g = ((float)(Green(c2)-Green(c1)))*interp+Green(c1);
      uint8_t b = ((float)(Blue(c2)-Blue(c1)))*interp+Blue(c1);
      if (charger_on && !runstop_on)
      {
        if (ColoredScanUpdate())
          ColoredScan(Color(r,g,b),Color(PX_OFF),2000);//Color((int)((float)r*0.1),(int)((float)g*0.1),(int)((float)b*0.1)),2000);
      }
      else
      {
        ColorSet(Color(r,g,b));
        show();
      }
    }
    else
    {
      clear();
      show();
    }
  }
};

//////////////////////////////////////////////////////////////////////////////////////////



LightBarManager::LightBarManager()
{
  pixels=0;
}

void LightBarManager::setupLightBarManager()
{
   pinMode(NEOPIXEL, OUTPUT);
   pixels = new LightBarPatterns(NUM_PIXELS, NEOPIXEL, NEO_GRB);
   pixels->begin(&sercom1, SERCOM1, SERCOM1_DMAC_ID_TX, NEOPIXEL, SPI_PAD_2_SCK_3, PIO_SERCOM_ALT);
}



void LightBarManager::step(bool boot_detected, bool runstop_on, bool charger_on, bool charging_required, bool runstop_led_on,float v_bat) 
{
  if (pixels)
  {
      pixels->ColoredBatteryLevel(v_bat, V_BAT_MIN, V_BAT_MAX, runstop_on, runstop_led_on, charger_on );
  }
}
