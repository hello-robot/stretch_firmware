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

//Slew a pixel color from bg to fg and back over duration_ms
class SlewPixel
{
  public:
  float r,b,g;
  int rf,rb,bf,bb,gf,gb;
  float d_pct;
  float pct;
  SlewPixel()
  {
  }
  void Configure(uint32_t color_bg, uint32_t color_fg, float duration_ms, float init_pct)
  {
    rf=Red(color_fg);
    rb=Red(color_bg);
    gf=Green(color_fg);
    gb=Green(color_bg);
    bf=Blue(color_fg);
    bb=Blue(color_bg);
    pct=init_pct;
    d_pct = 10/duration_ms;
    SetPct(init_pct);
  }
  void SetPct(float pct) //pass in 0-1.0. O.5 is FG color 0/1.0 is BG color
  {
    if(pct<0.5)
    {
      r=rb*(1-pct*2)+rf*pct*2;
      g=gb*(1-pct*2)+gf*pct*2;
      b=bb*(1-pct*2)+bf*pct*2;
    }
    else
    {
      pct=pct-0.5;
      r=rf*(1-pct*2)+rb*pct*2;
      g=gf*(1-pct*2)+gb*pct*2;
      b=bf*(1-pct*2)+bb*pct*2;
    }
  }
  uint32_t Step()
  {
    pct=min(1.0,pct+d_pct);
    SetPct(pct);
    if(pct==1.0)
      pct=0.0;
  }
  uint8_t Red(uint32_t color){return (color >> 16) & 0xFF;}
  uint8_t Green(uint32_t color){return (color >> 8) & 0xFF;}
  uint8_t Blue(uint32_t color){return color & 0xFF;}
};


class LightBarPatterns : public Adafruit_NeoPixel_ZeroDMA
{
    public:
    LightBarPatterns(uint16_t pixels, uint8_t pin, uint8_t type)
    :Adafruit_NeoPixel_ZeroDMA(pixels, pin, type)
    {
     
    }
    
    uint32_t Color1, Color2;  // What colors are in use
    float db;
    float brightness;
    float min_brightness;
    SlewPixel p0, p1, p2, p3;
    bool blink_ramp;
    unsigned long t_toggle_last;
    int blink_duration_ms;
  
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
    void RunstopBlink(uint32_t color)
    {
      Color1=color;
      ColorSet(Color1);
    }
    void RunstopBlinkUpdate(bool runstop_led_on)
    {
      if (runstop_led_on)
        ColorSet(Color1);
      else
      {
        clear();
        show();
      }
      
    }
//////////////////////////////////////////////////////////////////////
    void ColoredBlink(uint32_t color, float duration_ms, float min_b)
    {
      blink_duration_ms=duration_ms;
      db=255/(duration_ms/10.0);
      min_brightness=min_b;
      Color1=color;
      ColorSet(Color1);
      brightness=0;
      blink_ramp=true;
      t_toggle_last=0;
    }
    void ColoredBlinkUpdate()
    { 
      unsigned long t = time_manager.get_elapsed_time_ms();
      if (t-t_toggle_last>blink_duration_ms)
      {
        t_toggle_last=t;
        blink_ramp=!blink_ramp;
      }
      
      //allow negative brightness so can turn all the way off for a period
      if(brightness<255 && blink_ramp) //ramp up
          brightness=min(255,brightness+db);
      
      if(brightness>min_brightness && !blink_ramp) //ramp down
        brightness=max(min_brightness,brightness-db);
    setBrightness((uint8_t)max(0,brightness));
    show();
  }
//////////////////////////////////////////////////////////////////////  
  void ColoredScan(uint32_t color_bg,uint32_t color_fg,float duration_ms)
  {
    p0.Configure(color_bg, color_fg, duration_ms, 0.0);
    p1.Configure(color_bg, color_fg, duration_ms, 0.17);
    p2.Configure(color_bg, color_fg, duration_ms, 0.33);
    p3.Configure(color_bg, color_fg, duration_ms, 0.5);
  }
  void ColoredScanUpdate()
  {
    p0.Step();
    p1.Step();
    p2.Step();
    p3.Step();
    setPixelColor(0, Color(p0.r,p0.g,p0.b));
    setPixelColor(1, Color(p1.r,p1.g,p1.b));
    setPixelColor(2, Color(p2.r,p2.g,p2.b));
    setPixelColor(3, Color(p3.r,p3.g,p3.b));
    show();
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
   mode=OFF; 
    
}
void LightBarManager::step(bool not_booted, bool runstop_on, bool charger_on, bool charging_required, bool runstop_led_on) 
{
  if (pixels)
  {
    if(charging_required)//highest priority
    {
      if(mode!=CHARGING_REQUIRED)
      {
        pixels->ColoredBlink(pixels->Color(255,0,0),1000,-10);
        mode=CHARGING_REQUIRED;
      }
    }
    else 
    {
      if (not_booted) //second priority
      {
        if(mode!=BOOTING)
        {
          pixels->ColoredScan(pixels->Color(255,50,100),pixels->Color(0,0,0),2000);
          mode=BOOTING;
        }
      }
      else
      {
        if (charger_on) //third priority
        {
          if(runstop_on)
          {
            if(mode!=CHARGING_RUNSTOP_ON)
            {
              pixels->RunstopBlink(pixels->Color(255,191,0));
              mode=CHARGING_RUNSTOP_ON;
            }
          }
          else
          {
            if(mode!=CHARGING_RUNSTOP_OFF)
            {
              pixels->ColorSet(pixels->Color(255,191,0));
              mode=CHARGING_RUNSTOP_OFF;
            }
          }
        }
        else //fourth priority: normal operation
        {
          if(runstop_on)
          {
            if(mode!=NORMAL_RUNSTOP_ON)
            {
              pixels->RunstopBlink(pixels->Color(128,128,128));
              mode=NORMAL_RUNSTOP_ON;
            }
          }
          else
          {
            if(mode!=NORMAL_RUNSTOP_OFF)
            {
              pixels->ColorSet(pixels->Color(128,128,128));
              mode=NORMAL_RUNSTOP_OFF;
            }
          }
        }//4
      }
    }
    
    switch(mode){
      case OFF:
        pixels->Off();
        break;
      case BOOTING:
        pixels->ColoredScanUpdate();
        break;
      case CHARGING_RUNSTOP_ON:
      case NORMAL_RUNSTOP_ON:
        pixels->RunstopBlinkUpdate(runstop_led_on);
        break;
      case CHARGING_RUNSTOP_OFF:
      case NORMAL_RUNSTOP_OFF:
        break;
      case CHARGING_REQUIRED:
        pixels->ColoredBlinkUpdate();
        break;
    };
  }
}
