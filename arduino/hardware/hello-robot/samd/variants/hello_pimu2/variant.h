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
 * Analog pins 0 to 3
 */

#define PIN_SYS_VOLT          (0ul)
#define PIN_SYS_IMON          (PIN_SYS_VOLT + 1)
#define PIN_VTEMP             (PIN_SYS_VOLT + 2)
#define PIN_CHARGER_CURRENT   (PIN_SYS_VOLT + 3)


//declration needed for dependicies
static const uint8_t A0  = PIN_SYS_VOLT;

/*
* Digital Output Pins 4 to 15
*/
#define PIN_ARM_EN            (4ul)
#define PIN_LW_EN             (PIN_ARM_EN + 1)
#define PIN_RW_EN             (PIN_ARM_EN + 2)
#define PIN_CW_EN             (PIN_ARM_EN + 3)
#define PIN_EOA_EN            (PIN_ARM_EN + 4)
#define PIN_LIFT_EN           (PIN_ARM_EN + 5)
#define PIN_MOTORS_SYNC       (PIN_ARM_EN + 6)
#define PIN_MOTORS_RUNSTOP    (PIN_ARM_EN + 7)
#define PIN_RUNSTOP_LED       (PIN_ARM_EN + 8)
#define PIN_STS_LED           (PIN_ARM_EN + 9)
#define PIN_LATCH_CTRL        (PIN_ARM_EN + 10)
#define PIN_FAN_EN            (PIN_ARM_EN + 11)

/*
* Digital Input Pins 16 to 17
*/
#define PIN_CHARGER_CONNECT  (16ul)
#define PIN_RUNSTOP_IN       (PIN_CHARGER_CONNECT + 1)



/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SDA         (26u)
#define PIN_WIRE_SCL         (27u)
#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler

//Declration Needed for the I2C libarary
static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_DM          (19ul)
#define PIN_USB_DP          (20ul)

/*
* Neopixel SPI DMA Access
*/

#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_MOSI (18ul) //NeoPixel Output
#define PIN_SPI_SCK  (24ul) //See pin array these are defined as not a pin
#define PIN_SPI_MISO (25ul) //See pin array these are defined as not a pin
#define PERIPH_SPI  sercom0
#define PAD_SPI_TX  SPI_PAD_3_SCK_1
#define PAD_SPI_RX  SERCOM_RX_PAD_2 
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;
static const uint8_t MOSI  = PIN_SPI_MOSI;

//Not used pins needed for other dependcies
#define PIN_DAC0 (21ul)
#define PIN_DAC1 (22ul)
#define PIN_USB_HOST_ENABLE (23ul)

//Pimu firmware unused pins, just used for compling
#define PIN_IMU_RESET (28ul)
#define PIN_IMU_INT   (29ul)
#define PIN_BUZZER    (30ul)

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
