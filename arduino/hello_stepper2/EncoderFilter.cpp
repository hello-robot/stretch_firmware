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

#include "EncoderFilter.h"
#include "HelloController.h"

//based on https://github.com/mjbots/moteus/fw/motor_position.h
//see https://jpieper.com/2021/05/10/filtering-encoder-values-in-moteus/

EncoderFilter encoder_filter;

EncoderFilter::EncoderFilter()
{

}

float EncoderFilter::WrapCpr(float value, float cpr)
{
    int divisor = int(value / cpr);
    float mmod = value - divisor * cpr;
    if (mmod >= 0.0)
        return mmod;
    return mmod+cpr;
}


float EncoderFilter:: WrapBalancedCpr(float value, float cpr)
{
    return WrapCpr(value + 1.5 * cpr, cpr) - 0.5 * cpr;
}

void EncoderFilter::setupEncoderFilter()
{
    velocity=0;
    filtered_value=0;
    first_update = true;
    k2Pi=6.283185307179586;
    source_rate_hz=COMMUTATION_RATE_HZ; 
    dt=1/source_rate_hz;
    cpr=360.0; //ENCODER_CPR;
    float max_pll_hz = source_rate_hz / 10.0;
    pll_filter_hz=PLL_FILTER_HZ;
    pll_filter_hz =min(pll_filter_hz, max_pll_hz);
    float w_3db = pll_filter_hz * k2Pi;
    kp = 2.0 * w_3db;
    ki = w_3db * w_3db;
    max_counts_per_sample = 8;
    pll_filter_hz=0;
}

void EncoderFilter::stepFilter(float x)  //Pass in encoder reading
{

    if(first_update)
    {
      first_update=false;
      filtered_value = x;
      velocity = 0;
    }
    else
    {
        float unwrapped_error =-1*(filtered_value - x);
        float error =WrapBalancedCpr(unwrapped_error, cpr);

        filtered_value +=dt * kp * error;
        velocity +=dt * ki * error;
        //We don't let our velocity get beyond 1 revolution in 8 encode samples.
        float max_velocity = 0.125 * cpr / dt;
        if (velocity > max_velocity)
            velocity = max_velocity;
        else if(velocity < -1*max_velocity)
            velocity = -max_velocity;
        
    }
    filtered_value = WrapCpr(filtered_value, cpr);
}
