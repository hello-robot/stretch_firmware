/*
  -------------------------------------------------------------
  Hello Robot - Hello Stepper

  This code is derived from the Mechaduino project. 
  https://github.com/jcchurch13/Mechaduino-Firmware

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc.
  --------------------------------------------------------------
*/

#ifndef __ENCODER_FILTER_H__
#define  __ENCODER_FILTER_H__

#include "Common.h"

#define ENCODER_CPR 16384
#define PLL_FILTER_HZ 400 //Hardcode for now
class EncoderFilter {
   public: 
    EncoderFilter();
    void stepFilter(float x);
    void setupEncoderFilter();

    float velocity;
    float filtered_value;
    
  private:
    float WrapCpr(float value, float cpr);
    float WrapBalancedCpr(float value, float cpr);
    bool first_update;

    int max_counts_per_sample;
    float k2Pi;

    float source_rate_hz;
    float pll_filter_hz;
    float dt;

    float cpr;
    float kp;
    float ki;

};

extern EncoderFilter encoder_filter;
#endif
