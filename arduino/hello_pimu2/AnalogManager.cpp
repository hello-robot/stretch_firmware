/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/
#include "AnalogManager.h"
#include "Pimu.h"
#include "Common.h"

AnalogManager analog_manager;

//////////////////////// ANALOG READ ISR //////////////////////////////////////////////////

// Use the SAMD21's ISR to transfer ADC results to buffer array in memory


// //ADC0 Declrations
#define IDX_SYS_VOLT 0          //ADC0, AIN13 MUXPOS 0x0D
#define IDX_SYS_IMON 1         //ADC0, AIN7 MUXPOS 0x07 A4 on M4
#define IDX_CHARGER_CURRENT 2 //ADC0 AIN4 MUXPOS 0x04 A1 on M4

//ADC1 Declrations
#define IDX_VTEMP 0 //ADC1 AIN0 MUXPOS 0x09


//Default LPF Values
#define FACTORY_VOLTAGE_LPF 1
#define FACTORY_CURRENT_LPF 10
#define FACTORY_TEMP_LPF 1



AnalogManager::AnalogManager(){
  cliff_LPFa = 1.0; 
  cliff_LPFb = 0.0;
  voltage_LPFa = 1.0; 
  voltage_LPFb = 0.0;
  current_LPFa = 1.0; 
  current_LPFb = 0.0;
  temp_LPFa = 1.0; 
  temp_LPFb = 0.0;
  first_filter=1;

  adc_0_id = 0;
  adc_1_id = 0;

  adc_0_resultsReady = false;
  adc_1_resultsReady = false;
  first_config=1;



	//ADC0 PIN MUX MAP//
	adc_0_mux[IDX_SYS_VOLT] = 0x0D;
	adc_0_mux[IDX_SYS_IMON] = 0x07;
	adc_0_mux[IDX_CHARGER_CURRENT] = 0x04;

	//ADC1 PIN MUX MAP//
	adc_1_mux[IDX_VTEMP] = 0x09;

  }
  
//TODO: Write gains to Flash instead of hardcoding them here////
void AnalogManager::factory_config()
{
  Pimu_Config factory_config;
  factory_config.voltage_LPF = FACTORY_VOLTAGE_LPF;
  factory_config.current_LPF = FACTORY_CURRENT_LPF;

  voltage_LPFa = exp(factory_config.voltage_LPF*-2*3.14159/FS); // z = e^st pole mapping
  voltage_LPFb = (1.0-voltage_LPFa);

  current_LPFa = exp(factory_config.current_LPF*-2*3.14159/FS); // z = e^st pole mapping
  current_LPFb = (1.0-current_LPFa);

  temp_LPFa = exp(factory_config.temp_LPF*-2*3.14159/FS); // z = e^st pole mapping
  temp_LPFb = (1.0-temp_LPFa);

}
  
void AnalogManager::update_config(Pimu_Config * cfg_new, Pimu_Config * cfg_old)
{
  if (cfg_new->voltage_LPF!=cfg_old->voltage_LPF) 
  {
    voltage_LPFa = exp(cfg_new->voltage_LPF*-2*3.14159/FS); // z = e^st pole mapping
    voltage_LPFb = (1.0-voltage_LPFa);
  }
  if (cfg_new->current_LPF!=cfg_old->current_LPF) 
  {
    current_LPFa = exp(cfg_new->current_LPF*-2*3.14159/FS); // z = e^st pole mapping
    current_LPFb = (1.0-current_LPFa);
  }
  if (cfg_new->temp_LPF!=cfg_old->temp_LPF) 
  {
    temp_LPFa = exp(cfg_new->temp_LPF*-2*3.14159/FS); // z = e^st pole mapping
    temp_LPFb = (1.0-temp_LPFa);
  }
  if (first_config)
  {
    voltage = adc_0_Result[IDX_SYS_VOLT];
    current = adc_0_Result[IDX_SYS_IMON];
    current_charge = adc_0_Result[IDX_CHARGER_CURRENT];
    temp = adc_1_Result[IDX_VTEMP];
    cliff[0] = 0;
    cliff[1] = 0;
    cliff[2] = 0;
    cliff[3] = 0;
    first_config=0;
  }
}

    
void AnalogManager::step(Pimu_Status * stat, Pimu_Config * cfg)
{
  if (!(adc_0_resultsReady && adc_1_resultsReady))
		return;

  if (first_filter)
  {
    voltage = adc_0_Result[IDX_SYS_VOLT];
    current = adc_0_Result[IDX_SYS_IMON];
    current_charge = adc_0_Result[IDX_CHARGER_CURRENT];
    temp =    adc_1_Result[IDX_VTEMP];
    cliff[0] = 0;
    cliff[1] = 0;
    cliff[2] = 0;
    cliff[3] = 0;
    first_filter=false;
  }
  voltage = voltage*voltage_LPFa+voltage_LPFb*adc_0_Result[IDX_SYS_VOLT];
  current = current*current_LPFa+current_LPFb*adc_0_Result[IDX_SYS_IMON];
  current_charge = current_charge *current_LPFa +current_LPFb*adc_0_Result[IDX_CHARGER_CURRENT];
  temp =temp *temp_LPFa +temp_LPFb*adc_1_Result[IDX_VTEMP];
  cliff[0]= 0;
  cliff[1]= 0;
  cliff[2]=0;
  cliff[3]= 0;


  stat->cliff_range[0]=cliff[0]-cfg->cliff_zero[0];
  stat->cliff_range[1]=cliff[1]-cfg->cliff_zero[1];
  stat->cliff_range[2]=cliff[2]-cfg->cliff_zero[2];
  stat->cliff_range[3]=cliff[3]-cfg->cliff_zero[3];
  at_cliff[0] = stat->cliff_range[0]<cfg->cliff_thresh; //Neg is dropoff
  at_cliff[1] = stat->cliff_range[1]<cfg->cliff_thresh;
  at_cliff[2] = stat->cliff_range[2]<cfg->cliff_thresh;
  at_cliff[3] = stat->cliff_range[3]<cfg->cliff_thresh;


  adc_0_resultsReady = false;                              
  adc_1_resultsReady = false;

}

void AnalogManager::setupADC()
{
  //intilize clocks for ADC
  GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  while (GCLK->PCHCTRL[ADC0_GCLK_ID].bit.CHEN == 0);

  GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  while (GCLK->PCHCTRL[ADC1_GCLK_ID].bit.CHEN == 0);
  
	//Init ADC1///
	ADC1->INPUTCTRL.bit.MUXPOS = adc_1_mux[adc_1_id];   // Set the input control bit to start at positive mux input selection AIN0 page 1489
	while(ADC1->SYNCBUSY.bit.INPUTCTRL);                // Wait for the input control register to syncronize
	ADC1->SAMPCTRL.bit.SAMPLEN = 0x07;                  // Extend sampling time by SAMPCTRL ADC cycles (12 + 1 + 2)/3MHz = 5.0us sample time = 200kHz
	while(ADC1->SYNCBUSY.bit.SAMPCTRL);                // Wait for synchronization
	ADC1->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV256;      // Divide Clock ADC GCLK by 16 (48MHz/16 = 3MHz) (12 + 1 + 2)/3MHz = 5.0us sample time
  
	ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT |          // Set ADC resolution to 12 bits
					ADC_CTRLB_FREERUN;                			    // Set ADC to free run mode
	while(ADC1->SYNCBUSY.bit.CTRLB);                    // Wait for synchronization
	NVIC_SetPriority(ADC1_1_IRQn, 1);                   // Set the Nested Vector Interrupt Controller (NVIC) priority for the ADC to 1 0 does not work (highest)
	NVIC_EnableIRQ(ADC1_1_IRQn);                        // Connect the ADC to Nested Vector Interrupt Controller (NVIC)
 
	ADC1->INTENSET.reg = ADC_INTENSET_RESRDY;           //Set ADC1 Interupt set register to INTENSET_RESRDY
	// ADC1->CTRLA.bit.SLAVEEN = 1; //This allows ADC1 to be a slave to ADC0

  ADC0->INPUTCTRL.bit.MUXPOS = adc_0_mux[adc_0_id];         // Set the analog input to A0
	while(ADC0->SYNCBUSY.bit.INPUTCTRL);                // Wait for the input control register to syncronize
	ADC0->SAMPCTRL.bit.SAMPLEN = 0x07;                  // Extend sampling time by SAMPCTRL ADC cycles (12 + 1 + 2)/3MHz = 5.0us sample time = 200kHz
	while(ADC0->SYNCBUSY.bit.SAMPCTRL);                 // Wait for synchronization
	ADC0->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV256;      // Divide Clock ADC GCLK by 16 (48MHz/16 = 3MHz) (12 + 1 + 2)/3MHz = 5.0us sample time


	ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT |          // Set ADC resolution to 12 bits
					ADC_CTRLB_FREERUN;                // Set ADC to free run mode
	while(ADC0->SYNCBUSY.bit.CTRLB);                    // Wait for synchronization
	NVIC_SetPriority(ADC0_1_IRQn, 0);                   // Set the Nested Vector Interrupt Controller (NVIC) priority for the ADC to 1 0 does not work (highest)
	NVIC_EnableIRQ(ADC0_1_IRQn);                        // Connect the ADC to Nested Vector Interrupt Controller (NVIC)

	ADC0->INTENSET.reg = ADC_INTENSET_RESRDY;           //Set ADC0 Interupt set register to INTENSET_RESRDY
	ADC0->CTRLA.bit.ENABLE = 1;                         //Set the enable register to 1
	while(ADC0->SYNCBUSY.bit.ENABLE);                   // Wait for synchronization
	
	ADC1->CTRLA.bit.ENABLE = 1;                         //Set the enable register to 1
	while(ADC1->SYNCBUSY.bit.ENABLE);                   // Wait for synchronization
	
	
	ADC0->SWTRIG.bit.START = 1;                         // Initiate a software trigger to start an ADC conversion
	while(ADC0->SYNCBUSY.bit.SWTRIG);

		
	ADC1->SWTRIG.bit.START = 1;                         // Initiate a software trigger to start an ADC conversion
	while(ADC1->SYNCBUSY.bit.SWTRIG);
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

//ISR FOR ADC1
void ADC1_1_Handler()
{
  if (ADC1->INTFLAG.bit.RESRDY)                       // Check if the result ready (RESRDY) flag has been set
  {
    // digitalWrite(IMU_RESET, HIGH);
    ADC1->INTFLAG.bit.RESRDY = 1;                     // Clear the RESRDY flag
    while(ADC1->SYNCBUSY.bit.INPUTCTRL);                 // Wait for read synchronization
    analog_manager.adc_1_Result[analog_manager.adc_1_id] = ADC1->RESULT.reg;          // Read the result;
    analog_manager.adc_1_id++;
    if (analog_manager.adc_1_id==ADC_1_INPUTS)
    {
      analog_manager.adc_1_id=0;
      analog_manager.adc_1_resultsReady=true;
      // digitalWrite(IMU_RESET, LOW);
    }
    ADC1->CTRLA.bit.ENABLE = 0;                     // Disable the ADC
    while(ADC1->SYNCBUSY.bit.ENABLE);                // Wait for synchronization
    ADC1->INPUTCTRL.bit.MUXPOS = analog_manager.adc_1_mux[analog_manager.adc_1_id];         // Set the analog input channel
    while(ADC1->SYNCBUSY.bit.INPUTCTRL);                    // Wait for synchronization
    ADC1->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
    while(ADC1->SYNCBUSY.bit.ENABLE);                    // Wait for synchronization
    ADC1->SWTRIG.bit.START = 1;                         // Initiate a software trigger to start an ADC conversion
    while(ADC1->SYNCBUSY.bit.SWTRIG);                   // Wait for synchronization
  }
}