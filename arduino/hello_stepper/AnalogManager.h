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

#define NUM_ADC_INPUTS 1

class AnalogManager {    
  public: 
    AnalogManager();
    void update_config(Gains * cfg_new, Gains * cfg_old);
    void step();
    void setupADC();

    float voltage_LPFa; 
    float voltage_LPFb;
    float voltage;
    volatile uint16_t adcResult[NUM_ADC_INPUTS] = {};         // ADC results buffer
    uint8_t mux_map[NUM_ADC_INPUTS];
    uint8_t adc_input_id;
    bool first_read_done;
  private:
    bool first_filter;
    bool first_config;
};

extern AnalogManager analog_manager;

#endif
