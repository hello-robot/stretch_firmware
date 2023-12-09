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

#define NUM_ADC_INPUTS 9

class AnalogManager {    
  public: 
    AnalogManager();
    void update_config(Pimu_Config * cfg_new, Pimu_Config * cfg_old);
    void step(Pimu_Status * stat, Pimu_Config * cfg);
    void setupADC();
    void factory_config();
    
    float cliff_LPFa; 
    float cliff_LPFb;
    float voltage_LPFa; 
    float voltage_LPFb;
    float current_LPFa; 
    float current_LPFb;
    float temp_LPFa; 
    float temp_LPFb ;    
    float cliff[4];
    bool at_cliff[4];
    float voltage;
    float current;
    float current_charge;
    float current_efuse;
    float temp;
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
