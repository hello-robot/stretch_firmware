/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
    
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __IMU_FXOS8700_FXAS21002_H__
#define  __IMU_FXOS8700_FXAS21002_H__
#include "Common.h"
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Madgwick.h>

class IMU_FXOS8700_FXAS21002{
  public:
    IMU_FXOS8700_FXAS21002();
    void setupIMU();
    void stepIMU(IMU_Status * imu_status);
    void setIMUConfig(Pimu_Config * cfg_in, Pimu_Config * cfg);
    bool isIMUOrientationValid();
    Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
    Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
    Madgwick filter;
    float accel_max_log[3][8];
    int accel_max_idx=0;
    bool first_acc_filter=true;
    float accel_LPFa;
    float accel_LPFb;
    bool imu_valid;
  bool imu_calibration_rcvd;
  int imu_calibration_startup_cnt;
  float mag_offsets[3]            = { 0.0F, 0.0F, 0.02F };
  float mag_softiron_matrix[3][3];
  float mag_field_strength;
  float gyro_zero_offsets[3];
  float gravity_vector_scale;
  float rate_gyro_vector_scale;
};

extern IMU_FXOS8700_FXAS21002 imu_f;


#endif
