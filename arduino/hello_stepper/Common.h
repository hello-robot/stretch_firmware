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

#ifndef __COMMON_H__
#define __COMMON_H__

#include "Arduino.h"
/////////////////////////////////////////////////////////////////
//Version History
//Version History
// Stepper.V0.1: Initial production release for RE1
#define FIRMWARE_VERSION_HR "Stepper.v0.0.2p0"
#define BOARD_VERSION "Stepper.Joplin.V1"

/////////////////////////////////////////////////////////////////
#define RUNSTOP D0

#define RPC_SET_COMMAND  1
#define RPC_REPLY_COMMAND  2
#define RPC_GET_STATUS  3
#define RPC_REPLY_STATUS  4
#define RPC_SET_GAINS  5
#define RPC_REPLY_GAINS  6
#define RPC_LOAD_TEST 7
#define RPC_REPLY_LOAD_TEST 8
#define RPC_SET_TRIGGER  9
#define RPC_REPLY_SET_TRIGGER 10
#define RPC_SET_ENC_CALIB 11
#define RPC_REPLY_ENC_CALIB 12
#define RPC_READ_GAINS_FROM_FLASH 13
#define RPC_REPLY_READ_GAINS_FROM_FLASH 14
#define RPC_SET_MENU_ON 15
#define RPC_REPLY_MENU_ON 16
#define RPC_GET_STEPPER_BOARD_INFO 17
#define RPC_REPLY_STEPPER_BOARD_INFO 18
#define RPC_SET_MOTION_LIMITS 19
#define RPC_REPLY_MOTION_LIMITS 20

#define MODE_SAFETY 0
#define MODE_FREEWHEEL 1
#define MODE_HOLD 2
#define MODE_POS_PID 3
#define MODE_VEL_PID 4
#define MODE_POS_TRAJ 5
#define MODE_VEL_TRAJ 6
#define MODE_CURRENT 7
#define MODE_POS_TRAJ_INCR 8

#define DIAG_POS_CALIBRATED 1         //Has a pos mark trigger been recieved since powerup
#define DIAG_RUNSTOP_ON 2             //Is controller in runstop mode 
#define DIAG_NEAR_POS_SETPOINT 4      //Is pos controller within gains.pAs_d of setpoint
#define DIAG_NEAR_VEL_SETPOINT 8     //Is vel controller within gains.vAs_d of setpoint
#define DIAG_IS_MOVING 16             //Is measured velocity greater than gains.vAs_d
#define DIAG_AT_CURRENT_LIMIT 32      //Is controller current saturated
#define DIAG_IS_MG_ACCELERATING 64   //Is controler motion generator acceleration non-zero
#define DIAG_IS_MG_MOVING 128         //Is controller motion generator velocity non-zero
#define DIAG_CALIBRATION_RCVD 256      //Is the calibration table in flash
#define DIAG_IN_GUARDED_EVENT 512     //Guarded event occured
#define DIAG_IN_SAFETY_EVENT 1024     //Guarded event occured
#define DIAG_WAITING_ON_SYNC 2048     //Command rcvd but no sync trigger yet

#define TRIGGER_MARK_POS  1
#define TRIGGER_RESET_MOTION_GEN  2
#define TRIGGER_BOARD_RESET  4
#define TRIGGER_WRITE_GAINS_TO_FLASH 8
#define TRIGGER_RESET_POS_CALIBRATED 16
#define TRIGGER_POS_CALIBRATED 32

#define CONFIG_SAFE_MODE_HOLD 1
#define CONFIG_ENABLE_RUNSTOP 2
#define CONFIG_ENABLE_SYNC_MODE 4
#define CONFIG_ENABLE_GUARDED_MODE 8
#define CONFIG_FLIP_ENCODER_POLARITY 16
#define CONFIG_FLIP_EFFORT_POLARITY 32
/////////////////////////////////////////////////////////////////

//Note, to serialize to Linux must pack structs given use of sizeof()
//See https://arduino.stackexchange.com/questions/9899/serial-structure-data-transfer-between-an-arduino-and-a-linux-pc


struct __attribute__ ((packed)) Gains{
  float pKp;       //MODE_POS_PID: P gain
  float pKi;       //MODE_POS_PID: I gain
  float pKd;       //MODE_POS_PID: D gain
  float pLPF;      //MODE_POS_PID: D-term low pass filter roll-off
  float pKi_limit; //MODE_POS_PID: I saturation
  
  float vKp;       //MODE_VEL_PID: P gain
  float vKi;       //MODE_VEL_PID: I gain
  float vKd;       //MODE_VEL_PID: D gain
  float vLPF;      //MODE_VEL_PID: D-term low pass filter roll-off
  float vKi_limit; //MODE_VEL_PID: I saturation

  float vTe_d;      //MODE_VEL_TRAJ: Tracking thresh (deg)

  float iMax_pos;     //Current limit positive direction (A)
  float iMax_neg;     //Current limit negative direction (A)
  float phase_advance_d;     //Phase advance (deg)
  float pos_near_setpoint_d;    //Postion at setpoint thresh (deg)
  float vel_near_setpoint_d;    //Velocity at setpoint thresh (deg/s)

  float vel_status_LPF;   //Low pass filter roll-off for status velocity (Hz)
  float effort_LPF;   ///Low pass filter roll-off for status effort (Hz)
  float safety_stiffness; //0-1, for when safety mode is hold
  float i_safety_feedforward; //current (A), for when safety mode is hold
  uint8_t config;
};

struct __attribute__ ((packed)) MotionLimits{
  float pos_min;       //max range, post calibration
  float pos_max;       //min range, post calibration
};

struct __attribute__ ((packed)) Trigger{
  uint32_t data; 
  float    tdata;
};

struct __attribute__ ((packed)) Status{
  uint8_t mode;   //current control mode
  float effort;   //ticks, 1 tick = 12.95mA
  double pos;      //rad, wrapped
  float vel;      //rad/sec
  float err;       //controller error (inner loop)
  uint32_t diag;       //diagnostic codes     
  uint32_t timestamp; //us
  float debug; 
  uint32_t guarded_event; 
};

/////////////////////////////////////////////////////////////////
struct __attribute__ ((packed)) Command{
  uint8_t mode;
  float x_des;
  float v_des;
  float a_des;
  float stiffness;    //0-1
  float i_feedforward; //A
  float i_contact_pos; //A
  float i_contact_neg; //A
  uint8_t incr_trigger;
};

struct __attribute__ ((packed)) LoadTest{
  uint8_t data[1024];
};

struct __attribute__ ((packed)) EncCalib{
  uint8_t page_id; //Calib table is 16384 floats. Break up int 256 pages of 64 floats
  float page[64];
};

struct __attribute__ ((packed)) Stepper_Board_Info{
    char board_version[20];
    char firmware_version_hr[20];
};

/////////////////////////////////////////////////////////////////

#endif
