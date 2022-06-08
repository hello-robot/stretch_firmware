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

#define IN_4  6
#define IN_3  5
#define VREF_2 4
#define VREF_1 9
#define IN_2  7
#define IN_1  8

#ifdef HELLO
#define ledPin  0
//Pins for DRV8842
#define DRV8842_FAULT_A 17 //PA4
#define DRV8842_FAULT_B 18 //PA5
#define DRV8842_NSLEEP_A 38 //PA13
#define DRV8842_NSLEEP_B 2 //PA14
#else
#define ledPin  13
#endif

#define chipSelectPin A2 //output to chip select

#define step_pin 1
#define dir_pin 0
#define enable_pin 2

//for faster digitalWrite:
#define IN_1_HIGH() (REG_PORT_OUTSET0 = PORT_PA06)
#define IN_1_LOW() (REG_PORT_OUTCLR0 = PORT_PA06)
#define IN_2_HIGH() (REG_PORT_OUTSET0 = PORT_PA21)
#define IN_2_LOW() (REG_PORT_OUTCLR0 = PORT_PA21)
#define IN_3_HIGH() (REG_PORT_OUTSET0 = PORT_PA15)
#define IN_3_LOW() (REG_PORT_OUTCLR0 = PORT_PA15)
#define IN_4_HIGH() (REG_PORT_OUTSET0 = PORT_PA20)
#define IN_4_LOW() (REG_PORT_OUTCLR0 = PORT_PA20)

#ifdef HELLO
#define ledPin_HIGH() (REG_PORT_OUTSET0 = PORT_PA11)
#define ledPin_LOW() (REG_PORT_OUTCLR0 = PORT_PA11)
#else
#define ledPin_HIGH() (REG_PORT_OUTSET0 = PORT_PA17)
#define ledPin_LOW() (REG_PORT_OUTCLR0 = PORT_PA17)
#endif

#define CHIPSELECT_HIGH() (REG_PORT_OUTSET1 = PORT_PB09)
#define CHIPSELECT_LOW() (REG_PORT_OUTCLR1 = PORT_PB09)

#define ENABLE_PROFILE_IO    // Define to enable profiling I/O pins

#ifdef ENABLE_PROFILE_IO  
  #define TEST1   3

  #define TEST1_HIGH() (REG_PORT_OUTSET0 = PORT_PA09)
  #define TEST1_LOW() (REG_PORT_OUTCLR0 = PORT_PA09)

#else
  #define TEST1_HIGH()
  #define TEST1_LOW() 
#endif



#endif
