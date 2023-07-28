/*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html
  
  Copyright (c) 2020 by Hello Robot Inc.
  --------------------------------------------------------------
*/

#ifndef __COMMON_H__
#define __COMMON_H__

#include "Arduino.h"

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//Version History
// Protocol 0: Initial production release for RE1
// Protocol 1: Add support for long timestamps
// Version 0.2.0: Add support for RE2 (board variants)
// Version 0.2.2: Initial production release RE2 Mitski
// Version 0.2.3: Add trace function
// Version 0.2.4: Reorg timing to fix IRQ overruns
// Version 0.3.0: Move to updated trace and protocol P2
// Version 0.3.1: Added Watchdog timer (WDT) reset feature
// Version 0.4.0: Move to fast transport V1 and P3
// Version 0.5.0: Move to support for Transport V1
// Version 0.5.1: Fix trace rollover issue

#define FIRMWARE_VERSION "Wacc.v0.5.1p3"



/////////////////////////////////////////////////////////////////
#define RPC_SET_WACC_CONFIG 1
#define RPC_REPLY_WACC_CONFIG 2
#define RPC_GET_WACC_STATUS 3
#define RPC_REPLY_WACC_STATUS 4
#define RPC_SET_WACC_COMMAND 5
#define RPC_REPLY_WACC_COMMAND 6
#define RPC_GET_WACC_BOARD_INFO 7
#define RPC_REPLY_WACC_BOARD_INFO 8
#define RPC_READ_TRACE 9
#define RPC_REPLY_READ_TRACE 10

#define RPC_LOAD_TEST_PUSH 11
#define RPC_REPLY_LOAD_TEST_PUSH 12
#define RPC_LOAD_TEST_PULL 13
#define RPC_REPLY_LOAD_TEST_PULL 14


#define TRIGGER_BOARD_RESET  1
#define TRIGGER_ENABLE_TRACE 2
#define TRIGGER_DISABLE_TRACE 4

#define STATE_IS_TRACE_ON 1        //Is trace recording


/////////////////////////////////////////////////////////////////
//From hello_wacc/variants.h
#define LED PIN_LED
//Accelerometer
#define ACCEL_INT1 PIN_INT1
#define ACCEL_INT2 PIN_INT2


//Expansion Header
#define HEADER_SPI_SS SS
#define HEADER_SPI_SCK SCK
#define HEADER_SPI_MISO MISO
#define HEADER_SPI_MOSI MOSI
#define HEADER_I2C_SCL PIN_WIRE_SCL
#define HEADER_I2C_SDA PIN_WIRE_SDA
#define HEADER_ANA0 PIN_A0

#define BOARD_ID_0  PIN_BOARD_ID_0
#define BOARD_ID_1  PIN_BOARD_ID_1
#define BOARD_ID_2  PIN_BOARD_ID_2

/////////////////////////////////////////////////////////////////

//Note, to serialize to Linux must pack structs given use of sizeof()
//See https://arduino.stackexchange.com/questions/9899/serial-structure-data-transfer-between-an-arduino-and-a-linux-pc

struct __attribute__ ((packed)) Wacc_Command{
  uint8_t d2;
  uint8_t d3;
  uint32_t trigger;
};

struct __attribute__ ((packed)) Wacc_Config{
  uint8_t accel_range_g; //2, 4, 8, or 16 G
  float accel_LPF;
  float ana_LPF;
  uint8_t accel_single_tap_dur;     ////625 µs/LSB, Default is 50 (31ms)
  uint8_t accel_single_tap_thresh; //62.5 mg/LSB (that is, 0xFF = 16 g).Default is 20 (1.25g)
  float accel_gravity_scale;
};

struct __attribute__ ((packed)) Wacc_Status{
  float ax;	//Accelerometer AX
  float ay;	//Accelerometer AY
  float az;	//Accelerometer AZ
  int16_t a0; //expansion header analog in
  uint8_t d0; //expansion header digital in
  uint8_t d1; //expansion header digital in
  uint8_t d2; //expansion header digital out
  uint8_t d3; //expansion header digital out
  uint32_t single_tap_count; //Accelerometer tap count
  uint32_t state;
  uint64_t timestamp; //us
  uint32_t debug;
};


/////////////////////////////////////////////////////////////////

struct __attribute__ ((packed)) Wacc_Board_Info{
  char board_variant[20];
  char firmware_version[20];
};

struct __attribute__ ((packed)) LoadTest{
  uint8_t data[1024];
};

/////////////////////////////////////////////////////////////////






/////////////////////////////////////////////////////////////////

#endif
