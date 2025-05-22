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

#define ADC_0_INPUTS 3
#define ADC_1_INPUTS 4

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


    float voltage_5v0;
    float voltage_36v0;
    float voltage_20v0;

    float current_charger;
    float current_cpu;
    float current_rpi;
    
    float temp;
    float current_efuse;

    
    volatile uint16_t adc_0_Result[ADC_0_INPUTS] = {};  
    volatile uint16_t adc_1_Result[ADC_1_INPUTS] = {};

    uint8_t adc_0_mux[ADC_0_INPUTS];
    uint8_t adc_1_mux[ADC_1_INPUTS];

    uint8_t adc_0_id;
    uint8_t adc_1_id;

    volatile bool adc_0_resultsReady;
    volatile bool adc_1_resultsReady;

    bool first_read_done;
  private:
    bool first_filter;
    bool first_config;
};

extern AnalogManager analog_manager;

#endif
