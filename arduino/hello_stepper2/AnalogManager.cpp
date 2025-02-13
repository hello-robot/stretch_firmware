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

// Use the SAMD51's ISR to transfer ADC results to buffer array in memory


////ADC0 ID Declrations
#define IDX_ANA_V_BATT 0 //ADC0, AIN4 MUXPOS 0x04
#define IDX_ANA_V_TEMP 1 //ADC0, AIN3 MUXPOS 0x03 


AnalogManager::AnalogManager()
{
    voltage_LPFa = 1.0; 
    voltage_LPFb = 0.0;
    temp_LPFa = 1.0; 
    temp_LPFb = 0.0;
    first_filter=1;

    adc_0_id = 0;

    adc_0_resultsReady = false;
    first_config=1;

    //ADC0 PIN MUX MAP//
    adc_0_mux[IDX_ANA_V_BATT] = 0x04;
    adc_0_mux[IDX_ANA_V_TEMP] = 0x03;
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
    voltage = adc_0_Result[IDX_ANA_V_BATT];
    temp    = adc_0_Result[IDX_ANA_V_TEMP];
    first_config=0;
    }
}

void AnalogManager::step()
{
  if (!(adc_0_resultsReady))
      return;

  voltage = adc_0_Result[IDX_ANA_V_BATT];
  temp    = adc_0_Result[IDX_ANA_V_TEMP];

  adc_0_resultsReady = false;       
}

void AnalogManager::setupADC()
{

  ADC0->INPUTCTRL.bit.MUXPOS = adc_0_mux[adc_0_id];   // Set the analog input to A0
	while(ADC0->SYNCBUSY.bit.INPUTCTRL);                // Wait for the input control register to syncronize
	ADC0->SAMPCTRL.bit.SAMPLEN = 0x07;                  // Extend sampling time by SAMPCTRL ADC cycles (12 + 1 + 2)/3MHz = 5.0us sample time = 200kHz
	while(ADC0->SYNCBUSY.bit.SAMPCTRL);                 // Wait for synchronization
	ADC0->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV256;      // Divide Clock ADC GCLK by 16 (48MHz/16 = 3MHz) (12 + 1 + 2)/3MHz = 5.0us sample time


	ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT |          // Set ADC resolution to 12 bits
					ADC_CTRLB_FREERUN;                // Set ADC to free run mode
	while(ADC0->SYNCBUSY.bit.CTRLB);                    // Wait for synchronization
	NVIC_SetPriority(ADC0_1_IRQn, 1);                   // Set the Nested Vector Interrupt Controller (NVIC) priority for the ADC to 1 0 does not work (highest)
	NVIC_EnableIRQ(ADC0_1_IRQn);                        // Connect the ADC to Nested Vector Interrupt Controller (NVIC)

	ADC0->INTENSET.reg = ADC_INTENSET_RESRDY;           //Set ADC0 Interupt set register to INTENSET_RESRDY
	ADC0->CTRLA.bit.ENABLE = 1;                         //Set the enable register to 1
	while(ADC0->SYNCBUSY.bit.ENABLE);                   // Wait for synchronization
	
	ADC0->SWTRIG.bit.START = 1;                         // Initiate a software trigger to start an ADC conversion
	while(ADC0->SYNCBUSY.bit.SWTRIG);
}




//ISR FOR ADC0
void ADC0_1_Handler()
{
  if (ADC0->INTFLAG.bit.RESRDY)                       // Check if the result ready (RESRDY) flag has been set
  {
    // digitalWrite(IMU_RESET, HIGH);
    ADC0->INTFLAG.bit.RESRDY = 1;                     // Clear the RESRDY flag
    while(ADC0->SYNCBUSY.bit.INPUTCTRL);                 // Wait for read synchronization
    analog_manager.adc_0_Result[analog_manager.adc_0_id] = ADC0->RESULT.reg;          // Read the result;
    analog_manager.adc_0_id++;
    if (analog_manager.adc_0_id==ADC_0_INPUTS)
    {
      analog_manager.adc_0_id=0;
      analog_manager.adc_0_resultsReady=true;
      // digitalWrite(IMU_RESET, LOW);
    }
    ADC0->CTRLA.bit.ENABLE = 0;                     // Disable the ADC
    while(ADC0->SYNCBUSY.bit.ENABLE);                // Wait for synchronization
    ADC0->INPUTCTRL.bit.MUXPOS = analog_manager.adc_0_mux[analog_manager.adc_0_id];         // Set the analog input channel
    while(ADC0->SYNCBUSY.bit.INPUTCTRL);                    // Wait for synchronization
    ADC0->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
    while(ADC0->SYNCBUSY.bit.ENABLE);                    // Wait for synchronization
    ADC0->SWTRIG.bit.START = 1;                         // Initiate a software trigger to start an ADC conversion
    while(ADC0->SYNCBUSY.bit.SWTRIG);                   // Wait for synchronization
  }
}
