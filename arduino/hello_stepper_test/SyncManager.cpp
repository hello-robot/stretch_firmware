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

#include "SyncManager.h"
#include "HelloController.h"
#include "Controller.h"
#include "TimeManager.h"
///////////////////////// RUNSTOP & MOTOR SYNC ///////////////////////////
/*
* Monitor runstop line using an ISR for
* : Status sync : 2ms high pulse
* : Motor + status sync : 4ms high pulse
* : Activate runstop: >10ms high
* 
* The runstop ISR has highest priority to ensure accurate pulse measurement
* 
* Time measurement is by TC3 which rollsover at 100hz
* TC3 is reset at the start of a pulse
* Therefore can only measure pulse up to 10ms
* 
* The runstop is active when the digital input has been high over 10ms
* The digital input is pulled high internally such that a disconnected cable will make the runstop active
* The pulse polarity depends on the runstop state (active high pulse when runstop is not active)
 */


#define US_PER_TC3_TICK 1000000.0*8/48000000 //166ns resolution 8:1 prescalar
#define TC3_COUNT_PER_CYCLE (int)( round(48000000 / 8/ 100)) //60,000 or 100hz, so if motor sync line is high longer than 10ms then stop pulse measurement

//Status clock pulse is 2ms
//Motor sync pulse is 4ms
//Allow a little room for jitter. +75/-0 us is typical
#define SYNC_PULSE_STATUS_ONLY_MIN 10800     //1800/US_PER_TC3_TICK
#define SYNC_PULSE_STATUS_ONLY_MAX 13200     //2200/US_PER_TC3_TICK 
#define SYNC_PULSE_STATUS_MOTOR_MIN 22800       //3800/US_PER_TC3_TICK
#define SYNC_PULSE_STATUS_MOTOR_MAX 25200       //4200us/US_PER_TC3_TICK 


SyncManager sync_manager;

SyncManager::SyncManager()
{
  sync_duration_base_start=0;
  sync_duration_cntr_start=0;
  sync_duration_base_end=0;
  sync_duration_cntr_end=0;
  runstop_active=1;
  motor_sync_triggered=0;
  in_pulse=0;
  last_pulse_duration=-1;//N/A
  sync_mode_enabled = false;
}
    
//Called from runstop IO ISR on rising edge.
//Note, rising edge may be the start of a runstop event
//as well as start of a Status Sync of Motor Sync request
//This can cause the timestamp to be off at start of a runstop event
//Ok for now as timestamping of encoder when runstop on isn't meaningful    

////////////// Pulse Measurement ////////////////////
void sync_ISR() //Called on pin state change with highest priority (0)
{
  if (!sync_manager.in_pulse) //Start of pulse: rising edge when runstop not active, falling edge when runstop active
    sync_manager.start_pulse_measure();
  else
    sync_manager.end_pulse_measure();   
}

void SyncManager::start_pulse_measure() 
{
    in_pulse=1;
    TC3->COUNT16.COUNT.reg=0;
    enableTC3Interrupts();
    if (sync_mode_enabled)
    {
      memcpy((uint8_t *) (&stat_sync),(uint8_t *) (&stat_out),sizeof(Status));
      stat_sync.timestamp_last_sync=time_manager.current_time_us();
    }
}
    
void  SyncManager::end_pulse_measure() //Called from runstop IO ISR of falling edge
{
    int cntr = TC3->COUNT16.COUNT.reg; 
    disableTC3Interrupts();
    if (in_pulse) //Avoid spurious triggers at startup
    {
      if (sync_mode_enabled)
      {
        if (cntr>SYNC_PULSE_STATUS_MOTOR_MIN && cntr<SYNC_PULSE_STATUS_MOTOR_MAX)
          motor_sync_triggered=1;
      }
      last_pulse_duration=(int)(round(cntr*US_PER_TC3_TICK)); 
      stat.debug=last_pulse_duration;
      in_pulse=0;
    }
}

void TC3_Handler() {                // gets if the end of pulse isn't detected after 10ms from the start of pulse

  if (TC3->COUNT16.INTFLAG.bit.OVF == 1) // A counter overflow caused the interrupt
  {    
    //if (sync_manager.in_pulse)
    //  stat.debug++;
    sync_manager.in_pulse=0;
    TC3->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
    if (sync_manager.sync_mode_enabled)
      sync_manager.runstop_active = digitalRead(RUNSTOP);
  }
}

void  SyncManager::step() //Called at 1Khz from TC4 loop
{
  if (!sync_mode_enabled) //Poll runstop if not in sync mode
    runstop_active = digitalRead(RUNSTOP);
}
/////////////////////////////////////////////////////////////////



void SyncManager::enableTC3Interrupts() 
{  
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC3
  WAIT_TC16_REGS_SYNC(TC3)                      //wait for sync
}

void SyncManager::disableTC3Interrupts() 
{ 
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC3
  WAIT_TC16_REGS_SYNC(TC3)                      // wait for sync
}

void SyncManager::setupSyncManager() {  // configure the controller interrupt

  ////////////////////////// Setup TC3 ///////////////////////////

// Enable GCLK for TC2 and T3 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC2_TC3));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC3)                      // wait for sync

  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC3)

  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC3)

  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC3)

  TC3->COUNT16.CC[0].reg =  TC3_COUNT_PER_CYCLE;//TC3_COUNT_PER_CYCLE*10; //(int)( round(48000000 / FsCtrl / 2)); //0x3E72; //0x4AF0;
  WAIT_TC16_REGS_SYNC(TC3)

  TC3->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC3->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC3->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0

  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC3
  WAIT_TC16_REGS_SYNC(TC3)                      //wait for sync
  
   ////////////////////////// Setup Interrupts ///////////////////////////

//Set interrupt priority so runstop ISR garunteed to be caught without delay
  attachInterrupt(RUNSTOP, sync_ISR, CHANGE);

  NVIC_SetPriority(EIC_IRQn, 0);      
  NVIC_SetPriority(TC3_IRQn, 3);
 
  NVIC_EnableIRQ(TC3_IRQn);

}
