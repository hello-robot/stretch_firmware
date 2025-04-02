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

//Contains the Mechaduino parameter declarations

#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__



#define HELLO   //Turn on and off the Hello Codebase

#ifdef HELLO
#define firmware_version "0.1"            //firmware version
#define identifier "Hello Stepper"       // change this to help keep track of multiple mechaduinos (printed on startup)
#else
#define firmware_version "0.1.5"    //firmware version
#define identifier "x"              // change this to help keep track of multiple mechaduinos (printed on startup)
#endif

//----Current Parameters-----

extern volatile float Ts;
extern volatile float Fs;

extern volatile float pKp;
extern volatile float pKi;
extern volatile float pKd;
extern volatile float pLPF;


extern volatile float vKp;
extern volatile float vKi;
extern volatile float vKd;
extern volatile float vLPF;

extern const float lookup[];


extern volatile float pLPFa;
extern volatile float pLPFb;
extern volatile float vLPFa;
extern volatile float vLPFb;


extern const int spr; //  200 steps per revolution
extern const float aps; // angle per step
extern int cpr; //counts per rev
extern const float stepangle;

extern volatile float PA;  //

extern volatile float iMAX;
extern volatile float rSense;
extern volatile int uMAX;


extern const int sin_1[];

//Defines for pins:

#define IN_4    PIN_INA_4
#define IN_3    PIN_INA_3
#define VREF_2  PIN_VREF_B
#define VREF_1  PIN_VREF_A
#define IN_2    PIN_INA_2
#define IN_1    PIN_INA_1

#ifdef HELLO
#define ledPin  PIN_STS_LED
//Pins for DRV8262
#define DRV_FAULT         PIN_DRV_FAULT
#define DRV_TOFF          PIN_MCU_TOFF 
#define DRV_SLEEP         PIN_DRV_SLEEP
#define DRV_TOFF_SELECT   PIN_TOFF_SELECT
#define DRV_DECAY         PIN_MCU_DECAY
#endif

#define chipSelectPin PIN_SPI_SS //output to chip select


//for faster digitalWrite 
//PORTA pins are group 0
//PORTB pins are group 1
//OUTCLR: pins that are configured as outputs will be set to LOW
//OUTSET: pins that are configured as outputs will be set to HIGH
//OUTTGK: Toggles pins

//IN_A_1 PA11
#define IN_1_HIGH() (REG_PORT_OUTSET0 = PORT_PA11)
#define IN_1_LOW() (REG_PORT_OUTCLR0 = PORT_PA11)

//IN_A_2 PB10
#define IN_2_HIGH() (REG_PORT_OUTSET1 = PORT_PB10)
#define IN_2_LOW() (REG_PORT_OUTCLR1 = PORT_PB10)

//IN_B_1 PA15
#define IN_3_HIGH() (REG_PORT_OUTSET0 = PORT_PA15)
#define IN_3_LOW() (REG_PORT_OUTCLR0 = PORT_PA15)

//IN_B_2 PA14
#define IN_4_HIGH() (REG_PORT_OUTSET0 = PORT_PA14)
#define IN_4_LOW() (REG_PORT_OUTCLR0 = PORT_PA14)

//STS_LED PA13
#define ledPin_HIGH() (REG_PORT_OUTSET0 = PORT_PA13)
#define ledPin_LOW() (REG_PORT_OUTCLR0 = PORT_PA13)
#define ledPin_TOGGLE() (REG_PORT_OUTTGL0 = PORT_PA13)

//SPI_SS PB14
#define CHIPSELECT_HIGH() (REG_PORT_OUTSET1 = PORT_PB14)
#define CHIPSELECT_LOW() (REG_PORT_OUTCLR1 = PORT_PB14)

#endif
