/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __ANALOG_MANAGER_H__
#define  __ANALOG_MANAGER_H__

#include "Common.h"

#define ADC_0_INPUTS 2

class AnalogManager {    
  public: 
    AnalogManager();
    void update_config(Gains * cfg_new, Gains * cfg_old);
    void step();
    void setupADC();

    float voltage;
    float temp;

    volatile uint16_t adc_0_Result[ADC_0_INPUTS] = {};          // ADC results buffer
    uint8_t adc_0_mux[ADC_0_INPUTS];
    uint8_t adc_0_id;

    volatile bool adc_0_resultsReady;
  private:
    bool first_filter;
    bool first_config;
};

extern AnalogManager analog_manager;

#endif
