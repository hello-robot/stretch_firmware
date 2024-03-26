/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __COMMON_H__
#define __COMMON_H__

#include "Arduino.h"

/////////////////////////////////////////////////////////////////
//Version History

// Protocol 0: Initial production release for RE1
// Protocol 1: Add support for long timestamps
// Version 0.2.0: Add support for Variant 1 (BOARD_VARIANT_DEDICATED_SYNC, charge connect, Neopixel, etc)
// Version 0.2.1: Update lightbar UI
// Version 0.2.3: Initial production release RE2 Mitski
// Version 0.2.4: Bugfix lightbar voltage display
// Version 0.2.5: Initial production release for RE2 Nina
// Version 0.2.6: Add trace function
// Version 0.3.0: Move to updated trace and protocol P2
// Version 0.3.1: Added Watchdog timer (WDT) reset feture, halved trace buffer
// Version 0.4.0: Move to fast motor sync, status_aux, and P3
// Version 0.5.0: Move to support for Transport V1
// Version 0.5.1: Fix trace rollover issue
// Version 0.6.0: Production release for S3 and P4. Move to new IMU and current monitoring. (BOARD VARIANT=3 for new IMU)
// Version 0.6.2: added interrupts for imu data, added writing to system orientation record for IMU orientation to match older robots
// Version 0.6.3: incorporting charging detection class
// Version 0.7.0: added is_charger_charging state to pimu status
// Version 0.7.1  added tilt detection feature using cliff sensor and IMU data
// Version 0.8.0  Protocol change for over_tilt_type
#define FIRMWARE_VERSION "Pimu.v0.8.0p6"

#define FS 100 //Loop rate in Hz for TC5

/////////////////////////////////////////////////////////////////
#define RPC_SET_PIMU_CONFIG 1
#define RPC_REPLY_PIMU_CONFIG 2
#define RPC_GET_PIMU_STATUS 3
#define RPC_REPLY_PIMU_STATUS 4
#define RPC_SET_PIMU_TRIGGER 5
#define RPC_REPLY_PIMU_TRIGGER 6
#define RPC_GET_PIMU_BOARD_INFO 7
#define RPC_REPLY_PIMU_BOARD_INFO 8
#define RPC_SET_MOTOR_SYNC 9
#define RPC_REPLY_MOTOR_SYNC 10
#define RPC_READ_TRACE 11
#define RPC_REPLY_READ_TRACE 12
#define RPC_GET_PIMU_STATUS_AUX 13
#define RPC_REPLY_PIMU_STATUS_AUX 14
#define RPC_LOAD_TEST_PULL 15
#define RPC_REPLY_LOAD_TEST_PULL 16
#define RPC_LOAD_TEST_PUSH 17
#define RPC_REPLY_LOAD_TEST_PUSH 18

/////////////////Map Pins////////////////////////////////////////////////
//From hello_pimu/variants.h
#define ANA_V_BATT PIN_A0
#define ANA_CLIFF_0 PIN_A1
#define ANA_CLIFF_1 PIN_A2
#define ANA_CLIFF_2 PIN_A3
#define ANA_CLIFF_3 PIN_A4
#define ANA_TEMP PIN_A5
#define ANA_CURRENT PIN_A6 //Stretch 2 and earlier, shunt on battery supply
#define ANA_CURRENT_EFUSE PIN_A7//Stretch 3 and later, e-fuse measurement: PA11
#define ANA_CURRENT_CHARGE PIN_A8//Stretch 3 and later, current supplied by charger : PA10

#define RUNSTOP_SW D0
#define LED D3
#define RUNSTOP_LED D5
#define FAN_FET D2
#define BUZZER D1
#define IMU_RESET D6

#define BOARD_ID_0  PIN_BOARD_ID_0
#define BOARD_ID_1  PIN_BOARD_ID_1
#define BOARD_ID_2  PIN_BOARD_ID_2

//Variant 0
#define RUNSTOP_M0 D7
#define RUNSTOP_M1 D8
#define RUNSTOP_M2 D9
#define RUNSTOP_M3 D10
//Variant 1
#define RUNSTOP_OUT D7
#define SYNC_OUT D8
#define NEOPIXEL PIN_SPI_MOSI
#define CHARGER_CONNECTED PIN_CHARGER_CONNECTED

/////////////////////////////////////////////////////////////////
#define NUM_CLIFF 4

#define STATE_AT_CLIFF_0 1
#define STATE_AT_CLIFF_1 2
#define STATE_AT_CLIFF_2 4
#define STATE_AT_CLIFF_3 8
#define STATE_RUNSTOP_EVENT 16
#define STATE_CLIFF_EVENT 32
#define STATE_FAN_ON 64
#define STATE_BUZZER_ON 128
#define STATE_LOW_VOLTAGE_ALERT 256
#define STATE_OVER_TILT_ALERT 512
#define STATE_HIGH_CURRENT_ALERT 1024
#define STATE_CHARGER_CONNECTED 2048
#define STATE_BOOT_DETECTED 4096
#define STATE_IS_TRACE_ON 8192       //Is trace recording
#define STATE_IS_CHARGER_CHARGING 16384

#define TRIGGER_BOARD_RESET  1
#define TRIGGER_RUNSTOP_RESET  2
#define TRIGGER_CLIFF_EVENT_RESET 4
#define TRIGGER_BUZZER_ON  8
#define TRIGGER_BUZZER_OFF  16
#define TRIGGER_FAN_ON  32
#define TRIGGER_FAN_OFF  64
#define TRIGGER_IMU_RESET 128
#define TRIGGER_RUNSTOP_ON 256
#define TRIGGER_BEEP 512
#define TRIGGER_LIGHTBAR_TEST 1024
#define TRIGGER_ENABLE_TRACE 2048
#define TRIGGER_DISABLE_TRACE 4096
/////////////////////////////////////////////////////////////////

//Note, to serialize to Linux must pack structs given use of sizeof()
//See https://arduino.stackexchange.com/questions/9899/serial-structure-data-transfer-between-an-arduino-and-a-linux-pc


struct __attribute__ ((packed)) IMU_Status{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;
  float roll; //-180 to 180, rolls over
  float pitch; //-90 to  90, rolls over at 180
  float heading; //0-360.0, rolls over
  float qw;
  float qx;
  float qy;
  float qz;
  float bump;
};

/////////////////////////////////////////////////////////////////


struct __attribute__ ((packed)) Pimu_Config{
  float cliff_zero[NUM_CLIFF];
  float cliff_thresh;
  float cliff_LPF; //Hz rolloff
  float voltage_LPF; 
  float current_LPF;
  float temp_LPF;
  uint8_t stop_at_cliff;
  uint8_t stop_at_runstop;
  uint8_t stop_at_tilt;
  uint8_t stop_at_low_voltage;
  uint8_t stop_at_high_current;
  float mag_offsets[3];
  float mag_softiron_matrix[9];
  float gyro_zero_offsets[3];
  float rate_gyro_vector_scale;
  float gravity_vector_scale;
  float accel_LPF;
  float bump_thresh;
  float low_voltage_alert;
  float high_current_alert;
  float over_tilt_alert;
};

struct __attribute__ ((packed)) Pimu_Status{
  IMU_Status imu;
  float voltage;
  float current;
  float temp;
  float cliff_range[NUM_CLIFF];
  uint32_t state;      
  uint64_t timestamp; //us
  uint16_t bump_event_cnt;
  float debug;
  float current_charge;
  uint8_t over_tilt_type;

};

//Dummy struct for now, for future expansion
struct __attribute__ ((packed)) Pimu_Status_Aux{
  uint16_t foo;
};

struct __attribute__ ((packed)) Pimu_Trigger{
  uint32_t data;
};


struct __attribute__ ((packed)) Pimu_Motor_Sync_Reply{
  uint16_t motor_sync_cnt;
};

struct __attribute__ ((packed)) Pimu_Board_Info{
  char board_variant[20];
  char firmware_version[20];
};

/////////////////////////////////////////////////////////////////


struct __attribute__ ((packed)) LoadTest{
  uint8_t data[1024];
};



/////////////////////////////////////////////////////////////////

#endif
