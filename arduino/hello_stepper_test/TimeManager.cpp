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
  encoder_ts_base=0;
  encoder_ts_cntr=0;
  status_sync_ts_base=0;
  status_sync_ts_cntr=0;
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
  int cntr = TC4->COUNT16.COUNT.reg;
  uint64_t base = ts_base*US_PER_TC4_CYCLE;
  return base+(int)(round(cntr*US_PER_TC4_TICK));
}

void TimeManager::timestamp_encoder()
{
  encoder_ts_cntr=TC4->COUNT16.COUNT.reg;
  encoder_ts_base=ts_base;
}
void TimeManager::timestamp_status_sync()
{
  status_sync_ts_cntr=TC4->COUNT16.COUNT.reg;
  status_sync_ts_base=ts_base;
}

uint64_t TimeManager::get_encoder_timestamp()
{
  float delta = encoder_ts_cntr*US_PER_TC4_TICK;
  return encoder_ts_base*US_PER_TC4_CYCLE + delta;
}
uint64_t TimeManager::get_status_sync_timestamp()
{
  float delta = status_sync_ts_cntr*US_PER_TC4_TICK;
  return status_sync_ts_base*US_PER_TC4_CYCLE + delta;
}
