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

#ifndef _VARIANT_HELLO_PIMU_
#define _VARIANT_HELLO_PIMU_

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
#define PINS_COUNT           (27u)
#define NUM_DIGITAL_PINS     (18u)
#define NUM_ANALOG_INPUTS    (9u)
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

#define PIN_ANA_VBATT         (0ul)
#define PIN_ANA_CLIFF_2       (PIN_ANA_VBATT + 1)
#define PIN_ANA_CLIFF_3       (PIN_ANA_VBATT + 2)
#define PIN_ANA_TMP           (PIN_ANA_VBATT + 3)
#define PIN_ANA_CURRENT       (PIN_ANA_VBATT + 4)
#define PIN_ANA_CHARG_CURRENT (PIN_ANA_VBATT + 5)
#define PIN_ANA_EFUSE_IMON    (PIN_ANA_VBATT + 6)
#define PIN_ANA_CLIFF_0       (PIN_ANA_VBATT + 7)
#define PIN_ANA_CLIFF_1       (PIN_ANA_VBATT + 8)

//declration needed for dependicies
static const uint8_t A0  = PIN_ANA_VBATT;

/*
* Digital Input Pins
*/
#define PIN_PAUSE_SW  (9ul)
#define PIN_IMU_INT   (PIN_PAUSE_SW + 1)
#define PIN_BOARD_ID0 (PIN_PAUSE_SW + 2)
#define PIN_BOARD_ID1 (PIN_PAUSE_SW + 3)
#define PIN_BOARD_ID2 (PIN_PAUSE_SW + 4)
#define PIN_CHRG_STS  (PIN_PAUSE_SW + 5)

/*
* Digital Output Pins
*/
#define PIN_MCU_RUNSTOP   (15ul)
#define PIN_MOTORS_SYNC   (PIN_MCU_RUNSTOP + 1)
#define PIN_PAUSE_LED     (PIN_MCU_RUNSTOP + 2)
#define PIN_FAN           (PIN_MCU_RUNSTOP + 3)
#define PIN_BUZZER        (PIN_MCU_RUNSTOP + 4)
#define PIN_IMU_RESET     (PIN_MCU_RUNSTOP + 5)
#define PIN_STATUS_LED    (PIN_MCU_RUNSTOP + 6)
/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (22u)
#define PIN_WIRE_SCL         (23u)
#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler

//Declration Needed for the I2C libarary
static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_DM          (24ul)
#define PIN_USB_DP          (25ul)

/*
* Neopixel SPI DMA Access
*/

#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_MOSI (26ul)
#define PIN_SPI_SCK  (30ul) //See pin array these are defined as not a pin
#define PIN_SPI_MISO (31ul) //See pin array these are defined as not a pin
#define PERIPH_SPI  sercom3
#define PAD_SPI_TX  SPI_PAD_0_SCK_1
#define PAD_SPI_RX  SERCOM_RX_PAD_2 
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;
static const uint8_t MOSI  = PIN_SPI_MOSI;

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

