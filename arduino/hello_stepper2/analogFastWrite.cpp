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

//187kHz PWM implementation.  Stock analogWrite is much slower and is very audible!

#include "Arduino.h"
#include "wiring_private.h"


#ifdef __cplusplus
extern "C" {
#endif


static int _readResolution = 10;
static int _ADCResolution = 10;
static int _writeResolution = 8;


/////////////////////////////////////////////////////////////////////////////
// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_8(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_8(Tc* TCx) {
  while (TCx->COUNT8.SYNCBUSY.bit.ENABLE);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}


static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to) {
    return value;
  }
  if (from > to) {
    return value >> (from-to);
  }
  return value << (to-from);
}


// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.

void setup_pwm_pin(uint32_t pin)
{
  PinDescription pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;

  uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);

  pinPeripheral(pin, PIO_TIMER_ALT);

  //Generick clock IDs Stepper VREF is TCC0_GCLK_ID
  uint16_t GCLK_PCHCTRL_IDs[] = 
  {
    TCC0_GCLK_ID, // TCC0
    TCC1_GCLK_ID, // TCC1
    TCC2_GCLK_ID, // TCC2
    TC0_GCLK_ID,  // TC0
    TC1_GCLK_ID,  // TC1
    TC2_GCLK_ID,  // TC2
    TC3_GCLK_ID,  // TC3
    TC4_GCLK_ID,  // TC4
  };
    
  uint8_t gclkId = GCLK_PCHCTRL_IDs[tcNum];
  //GCLK1 is 48MHz
  GCLK->PCHCTRL[gclkId].reg = (uint16_t) (GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN);
  while (GCLK->PCHCTRL[gclkId].bit.CHEN == 0);

  // -- Configure TCC
  Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
  // Disable TCCx
  TCCx->CTRLA.bit.ENABLE = 0;
  syncTCC(TCCx);
  // Set TCx as normal PWM
  TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
  syncTCC(TCCx);
  // Set PER to maximum counter value (resolution : 0xFF)
  TCCx->PER.reg = 0xFF; //change to 0x43FF for 10 bit... must also change mapping above
  syncTCC(TCCx);
  // Enable TCCx
  TCCx->CTRLA.bit.ENABLE = 1;
  syncTCC(TCCx);
}

void analogFastWrite(uint32_t pin, uint32_t value)
{
  PinDescription pinDesc = g_APinDescription[pin];
  uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
  
  //Adjust value
  Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
  TCCx->CTRLBSET.bit.LUPD = 1;
  syncTCC(TCCx);
  TCCx->CC[tcChannel].reg = (uint32_t) value;
  syncTCC(TCCx);
  TCCx->CTRLBCLR.bit.LUPD = 1;
  syncTCC(TCCx);
}

#ifdef __cplusplus
}
#endif
