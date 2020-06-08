/*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc Spi

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
// Wacc.V0.1: Initial production release for RE1
#define FIRMWARE_VERSION "Wacc.v0.0.1pMySPI"
#define BOARD_VERSION "Wacc.Guthrie.V1"

/////////////////////////////////////////////////////////////////
#define RPC_SET_WACC_CONFIG 1
#define RPC_REPLY_WACC_CONFIG 2
#define RPC_GET_WACC_STATUS 3
#define RPC_REPLY_WACC_STATUS 4
#define RPC_SET_WACC_COMMAND 5
#define RPC_REPLY_WACC_COMMAND 6
#define RPC_GET_WACC_BOARD_INFO 7
#define RPC_REPLY_WACC_BOARD_INFO 8
#define TRIGGER_BOARD_RESET  1

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

/////////////////////////////////////////////////////////////////
struct __attribute__ ((packed)) Calc_Command{
  float var1;
  float var2;
  uint8_t op;
};
struct __attribute__ ((packed)) Calc_Status{
  float result;
};
/////////////////////////////////////////////////////////////////

//Note, to serialize to Linux must pack structs given use of sizeof()
//See https://arduino.stackexchange.com/questions/9899/serial-structure-data-transfer-between-an-arduino-and-a-linux-pc

struct __attribute__ ((packed)) Wacc_Command{
  Calc_Command calc;
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
  Calc_Status calc;
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
  uint32_t timestamp; //ms, overflows every 50 days
  uint32_t debug;
};

struct __attribute__ ((packed)) Wacc_Board_Info{
  char board_version[20];
  char firmware_version[20];
};
/////////////////////////////////////////////////////////////////






/////////////////////////////////////////////////////////////////

#endif
