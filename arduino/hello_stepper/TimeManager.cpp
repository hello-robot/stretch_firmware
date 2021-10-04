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

#include "TimeManager.h"
#include "HelloController.h"
#include "Controller.h"


/* 
 * As micros() doesn't work in ISR we track timing using TC4
 * TC4 drives the main control loop at 1Khz
 * We count TC4 overflows and compute a uint64_t timestamp that is rollover proof
 * The current timestamp is in uS since the last zero_clock() call
 * , which is called aperiodically at approximately 100hz from the TC4 loop.
 * The encoder is read in the TC5 loop and timestamped upon the SPI read to the sensor.
 * Upon a zero_clock() all timestamps are adjusted relative to the new zero.
 * , therefore a timestamp can be negative relative to the zero_clock
 * */
 
//Todo: Move to TCCO pulse measurement
//See https://forum.arduino.cc/index.php?topic=396804.30

TimeManager time_manager;

TimeManager::TimeManager()
{
  ts_base=0; 
  dt_ms= 1000.0/TC4_LOOP_RATE;
}

void TimeManager::setupTimeManager()
{

}

uint32_t TimeManager::get_elapsed_time_ms() //Avoid using millis() in ISR
{
  return (uint32_t)((float)(dt_ms*(float)ts_base));
}

uint64_t TimeManager::current_time_us()
{
  uint64_t base;
  int cntr = TC4->COUNT16.COUNT.reg;
  if (TC4->COUNT16.INTFLAG.bit.OVF == 1) //Catch the case that there's an IRQ waiting to be handled
    base = (ts_base+1)*US_PER_TC4_CYCLE;
  else
    base = ts_base*US_PER_TC4_CYCLE;
  return base+(int)(round(cntr*US_PER_TC4_TICK));
}
