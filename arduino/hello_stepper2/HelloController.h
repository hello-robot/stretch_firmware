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

#ifndef __HELLO_CONTROLLER_H__
#define  __HELLO_CONTROLLER_H__

#include "Common.h"




extern volatile bool hello_interface;

extern void setupHelloController();
extern void stepHelloController();
extern void stepHelloCommutation();
extern void stepHelloControllerRPC();
extern void toggle_led(int rate_ms);
extern void enableMotorDrivers();
extern void setMotorDecay(uint8_t decay);
extern void setTOFF(uint8_t toff);

extern float debug;


extern void enableTCInterrupts();
extern void disableTCInterrupts();
extern void setupTCInterrupts();

extern Status stat, stat_out;
extern volatile int dirty_cmd;

extern uint8_t    BOARD_VARIANT;
extern uint8_t    BOARD_VARIANT_DRV8842;
extern uint8_t    BOARD_VARIANT_PIN_RUNSTOP;
extern void setupBoardVariants();

extern void setupWDT( uint8_t period);
extern void resetWDT();
extern void WDTsync();
extern void systemReset();
extern void disableWDT();

extern uint16_t drv8262_min_vref;
extern float k_calibration_step;

#define CLOCK_RATE_HZ 48000000 //For SAMD51, actual clock is 120Mhz but using generic clock 1 (48M) for timing
#define WDT_TIMEOUT_PERIOD 11 //ms range 0-11ms

//Timer 4 simply steps the TimeManager
#define TC4_LOOP_RATE 1000                                                    //Update rate of control loop Hz
#define TC4_COUNT_PER_CYCLE (int)( round(CLOCK_RATE_HZ / 2/ TC4_LOOP_RATE))  //24,000 at 1Khz, 2:1 prescalar TC4 is 32bit timer 
#define US_PER_TC4_CYCLE 1000000/TC4_LOOP_RATE                                //1000 at 1KHz
#define US_PER_TC4_TICK 1000000.0*2/CLOCK_RATE_HZ                            //41.6ns resolution

//Timer 5 updates the commutation
#define TC5_LOOP_RATE 10000   //Commutation loop rate ((hz)    
#define COMMUTATION_RATE_HZ TC5_LOOP_RATE                                            
#define CONTROL_RATE_HZ 5000  //Control loop rate
#define CONTROL_LOOP_DIV TC5_LOOP_RATE/ CONTROL_RATE_HZ     //Downsample to control loop rate
#define MS_LOOP_RATE 10
#define CONTROL_TICKS_PER_MS (int)(CONTROL_RATE_HZ/1000)
#define TC5_COUNT_PER_CYCLE (int)( round(CLOCK_RATE_HZ / 1/ TC5_LOOP_RATE))   //960 at 50Khz, 1:1 prescalar TC5 is 32bit timer. Ideally no rounding/remainder in division.
#define US_PER_TC5_CYCLE 1000000/TC5_LOOP_RATE                                //20 at 50KHz
#define US_PER_TC5_TICK 1000000.0*1/CLOCK_RATE_HZ                             //20.8ns resolution

#endif
