/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_HELLO_STEPPER_
#define _VARIANT_HELLO_STEPPER_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK        (F_CPU)

#define VARIANT_GCLK0_FREQ (F_CPU)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (26u)
#define NUM_DIGITAL_PINS     (16u)
#define NUM_ANALOG_INPUTS    (2u)
#define NUM_ANALOG_OUTPUTS   (0u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

/*
 * Analog pins
 */

#define PIN_VBAT_VOLT         (0ul)
#define PIN_TEMP              (PIN_ANA_VBATT + 1)

//declration needed for dependicies
static const uint8_t A0  = PIN_VBAT_VOLT;

/*
* Digital Input Pins
*/
#define PIN_BOARD_ID0   (2ul)
#define PIN_BOARD_ID1   (PIN_BOARD_ID0 + 1)
#define PIN_BOARD_ID2   (PIN_BOARD_ID0 + 2)
#define PIN_FAULT_A     (PIN_BOARD_ID0 + 3)
#define PIN_FAULT_B     (PIN_BOARD_ID0 + 4)
#define PIN_MCU_RUNSTOP (PIN_BOARD_ID0 + 5)
#define PIN_MCU_SYNC    (PIN_BOARD_ID0 + 6)

/*
* Digital Output Pins
*/
#define PIN_VREF_A       (9ul)
#define PIN_VREF_B       (PIN_VREF_A + 1)
#define PIN_INA_1        (PIN_VREF_A + 2)
#define PIN_INA_2        (PIN_VREF_A + 3)
#define PIN_INB_1        (PIN_VREF_A + 4)
#define PIN_INB_2        (PIN_VREF_A + 5)
#define PIN_STS_LED      (PIN_VREF_A + 6)
#define PIN_MCU_SLEEP_A  (PIN_VREF_A + 7)
#define PIN_MCU_SLEEP_B  (PIN_VREF_A + 8)
#define PIN_MCU_DECAY    (PIN_VREF_A + 9)
#define PIN_MCU_BREAK    (PIN_VREF_A + 10)
#define PIN_DECAY_SELECT (PIN_VREF_A + 11)


/*
* SPI Interface
*/

#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_SS   (21ul) 
#define PIN_SPI_MOSI (22ul)
#define PIN_SPI_SCK  (23ul) 
#define PIN_SPI_MISO (24ul) 

#define PERIPH_SPI  sercom4
#define PAD_SPI_TX  SPI_PAD_0_SCK_1
#define PAD_SPI_RX  SERCOM_RX_PAD_3
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;
static const uint8_t MOSI  = PIN_SPI_MOSI;

/*
 * USB
 */
#define PIN_USB_DM          (25ul)
#define PIN_USB_DP          (26ul)

/*
 * Test Pins for profiling and other uses
 */
#define PIN_TEST            (32ul)

/*
 * Wire Interfaces not used needed for compiler
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (30uL) //See pin array these are defined as not a pin
#define PIN_WIRE_SCL         (31uL) //See pin array these are defined as not a pin
#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler

//Declration Needed for the I2C libarary
static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

//Not used pins needed for other dependcies
#define PIN_DAC0 (27ul)
#define PIN_DAC1 (28ul)
#define PIN_USB_HOST_ENABLE (29ul)

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;



#if !defined(VARIANT_QSPI_BAUD_DEFAULT)
  // TODO: meaningful value for this
  #define VARIANT_QSPI_BAUD_DEFAULT 5000000
#endif

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

// extern Uart Serial1;

#endif
#define Serial  SerialUSB
// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         SerialUSB
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
// #define SERIAL_PORT_HARDWARE        Serial1
// #define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_FEATHER_M4_ */

