/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/
#include "AnalogManager.h"
#include "Common.h"
#include "HelloController.h"

AnalogManager analog_manager;

//////////////////////// ANALOG READ ISR //////////////////////////////////////////////////

// Use the SAMD21's ISR to transfer ADC results to buffer array in memory


#define IDX_ANA_V_BATT 0 //AIN0 MUXPOS 0x00
#define FS 100 //Loop rate in Hz for TC5

AnalogManager::AnalogManager(){
  voltage_LPFa = 1.0; 
  voltage_LPFb = 0.0;
  first_filter=1;
  first_read_done=0;
  first_config=1;
  adc_input_id=0;

  mux_map[IDX_ANA_V_BATT]=0x00;
  }
  

void AnalogManager::update_config(Gains * cfg_new, Gains * cfg_old)
{
    if (cfg_new->voltage_LPF!=cfg_old->voltage_LPF) 
    {
    voltage_LPFa = exp(cfg_new->voltage_LPF*-2*3.14159/FsCtrl); // z = e^st pole mapping
    voltage_LPFb = (1.0-voltage_LPFa);
    }

    if (first_config)
    {
    voltage = adcResult[IDX_ANA_V_BATT];
    first_config=0;
    }
}

void AnalogManager::step()
{
  if (!first_read_done)
    return;

  if (first_filter)
  {
    voltage = adcResult[IDX_ANA_V_BATT];
    first_filter=false;
  }

  voltage = voltage * voltage_LPFa +  voltage_LPFb* adcResult[IDX_ANA_V_BATT];
}

void AnalogManager::setupADC()
{
  //ADC ISR setup
  ADC->INPUTCTRL.bit.MUXPOS = mux_map[adc_input_id];                
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
  //SAMPLEN 5:0 bits, sample time =(samplen+1)*(clk_adc/2)
  ADC->SAMPCTRL.bit.SAMPLEN = 0x07;                  // Set max Sampling Time Length to 4x ADC clock pulse (42us)
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |      // Divide Clock ADC GCLK by 512 (48MHz/512 = 93.7kHz)
                   ADC_CTRLB_RESSEL_10BIT |          // Set the ADC resolution to 10 bits (use ADC_CTRLB_RESSEL_12BIT for 12)
                   ADC_CTRLB_FREERUN;                // Set the ADC to free run
 
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization 
  NVIC_SetPriority(ADC_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for the ADC to 0 (highest)
  NVIC_EnableIRQ(ADC_IRQn);         // Connect the ADC to Nested Vector Interrupt Controller (NVIC)
  ADC->INTENSET.reg = ADC_INTENSET_RESRDY;           // Generate interrupt on result ready (RESRDY)
  ADC->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
  ADC->SWTRIG.bit.START = 1;                         // Initiate a software trigger to start an ADC conversion
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
}



//ISR
void ADC_Handler()
{
  if (ADC->INTFLAG.bit.RESRDY)                       // Check if the result ready (RESRDY) flag has been set
  {
    ADC->INTFLAG.bit.RESRDY = 1;                     // Clear the RESRDY flag
    while(ADC->STATUS.bit.SYNCBUSY);                 // Wait for read synchronization
    analog_manager.adcResult[analog_manager.adc_input_id] = ADC->RESULT.reg;          // Read the result;
    analog_manager.adc_input_id++;
    if (analog_manager.adc_input_id==NUM_ADC_INPUTS)
    {
      analog_manager.adc_input_id=0;
      analog_manager.first_read_done=1;
    }
    ADC->CTRLA.bit.ENABLE = 0;                     // Disable the ADC
    while(ADC->STATUS.bit.SYNCBUSY);               // Wait for synchronization
    ADC->INPUTCTRL.bit.MUXPOS = analog_manager.mux_map[analog_manager.adc_input_id];         // Set the analog input channel
    while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
    ADC->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
    while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
  }
}
